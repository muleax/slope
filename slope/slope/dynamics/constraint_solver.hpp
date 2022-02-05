#pragma once
#include "slope/dynamics/rigid_body.hpp"
#include "slope/containers/vector.hpp"
#include "slope/containers/array.hpp"
#include "slope/thread/task_executor.hpp"

namespace slope {

struct ConstraintGeom {
    vec3 p1;
    vec3 p2;
    vec3 axis;

    float pos_error() const { return axis.dot(p1 - p2); }
};

enum class ConstraintGroup : int {
    General = 0,
    Friction1D = 1,
    Friction2D = 2,
    FrictionCone = 3,
    Count
};

class ConstraintId {
public:
    ConstraintId() = default;
    ConstraintId(int raw) : m_raw(raw) {}
    ConstraintId(ConstraintGroup group, int index) : m_raw((int)group | index << 2) {}

    bool            is_valid() const { return m_raw >= 0; }
    ConstraintGroup group() const { return ConstraintGroup(m_raw & 3); }
    int             index() const { return m_raw >> 2; }
    int             raw() const { return m_raw; }

    operator int() const { return m_raw; }

    void operator++() { *this = {group(), index() + 1}; }

private:
    int m_raw = -1;
};

using ConstraintIds = std::pair<ConstraintId, ConstraintId>;

struct Constraint {
    static Constraint generic(
            RigidBody* body1, RigidBody* body2, const ConstraintGeom& geom,
            float pos_error, float min_bound, float max_bound);

    static Constraint bilateral(RigidBody* body1, RigidBody* body2, const ConstraintGeom& geom);
    static Constraint bilateral(RigidBody* body1, const ConstraintGeom& geom);

    static Constraint unilateral(RigidBody* body1, RigidBody* body2, const ConstraintGeom& geom);
    static Constraint unilateral(RigidBody* body1, const ConstraintGeom& geom);

    static Constraint stabilized_bilateral(RigidBody* body1, RigidBody* body2, const ConstraintGeom& geom);
    static Constraint stabilized_bilateral(RigidBody* body1, const ConstraintGeom& geom);

    static Constraint stabilized_unilateral(RigidBody* body1, RigidBody* body2, const ConstraintGeom& geom);
    static Constraint stabilized_unilateral(RigidBody* body1, const ConstraintGeom& geom);

    RigidBody*  body1 = nullptr;
    RigidBody*  body2 = nullptr;

    vec3        jacobian1[2];
    vec3        jacobian2[2];

    float       min_bound = -FLOAT_MAX;
    float       max_bound = FLOAT_MAX;

    float       init_lambda = 0.f;

    float       cfm = 0.f;
    float       erp = 0.2f;
    float       unilateral_penetration = 0.01f;
    float       pos_error = 0.f;
};

// Projected Gauss-Seidel constraint solver
class ConstraintSolver
{
public:
    struct Config {
        bool        use_simd = false;
        // Successive over-relaxation
        float       sor = 1.f;
        int         iteration_count = 10;
    };

    ConstraintSolver();
    virtual ~ConstraintSolver() = default;

    Config&         config() { return m_config; }
    const Config&   config() const { return m_config; }

    void            set_time_interval(float value);
    float           time_interval() const { return m_dt; }

    void            register_body(RigidBody* body);

    ConstraintId    allocate(ConstraintGroup group)
    {
        auto& group_data = m_groups[(int)group];
        ConstraintId constr_id = { group, group_data.size };

        group_data.size++;
        if (group_data.constraints.size() < group_data.size) {
            group_data.constraints.emplace_back();
            group_data.lambda.emplace_back();
        }
        return constr_id;
    }

    ConstraintIds    allocate_pair(ConstraintGroup group)
    {
        return { allocate(group), allocate(group) };
    }

    ConstraintIds   allocate_range(ConstraintGroup group, int count)
    {
        auto& group_data = m_groups[(int)group];
        ConstraintId begin_id = { group, group_data.size };

        group_data.size += count;

        if (group_data.constraints.size() < group_data.size) {
            group_data.constraints.resize(group_data.size);
            group_data.lambda.resize(group_data.size);
        }

        ConstraintId end_id = { group, group_data.size };
        return {begin_id, end_id};
    }

    void            setup_constraint(ConstraintId constr_id, const Constraint& c)
    {
        auto& data = basic_setup(constr_id, c);
        data.min_bound = c.min_bound;
        data.max_bound = c.max_bound;
        data.cfm_inv_dt = c.cfm * m_inv_dt;

        // TODO: reconsider
        if (c.min_bound == 0.f) {
            // unilateral case
            if (c.pos_error > c.unilateral_penetration)
                data.bg_error = (c.pos_error - c.unilateral_penetration) * c.erp;
            else
                data.bg_error = c.pos_error - c.unilateral_penetration;
        } else {
            // bilateral case
            data.bg_error = c.pos_error * c.erp;
        }
    }

    void            setup_friction_1d(ConstraintId constr_id, const Constraint& c, float friction_ratio, ConstraintId normal_constr_id)
    {
        auto& data = basic_setup(constr_id, c);

        data.cfm_inv_dt = c.cfm * m_inv_dt;
        data.bg_error = 0.f;

        data.friction_ratio = friction_ratio;
        data.normal_constr_idx = normal_constr_id.index();
    }

    void            setup_friction_2d(ConstraintIds constr_ids, const Constraint& c1, const Constraint& c2, vec2 friction_ratio, ConstraintId normal_constr_id)
    {
        auto setup_constraint = [this, normal_constr_id](ConstraintData& data, const Constraint& c, float friction_ratio) {
            data.cfm_inv_dt = c.cfm * m_inv_dt;
            data.bg_error = 0.f;

            data.friction_ratio = friction_ratio;
            data.normal_constr_idx = normal_constr_id.index();
        };

        auto& data1 = basic_setup(constr_ids.first, c1);
        setup_constraint(data1, c1, friction_ratio.x);

        auto& data2 = basic_setup(constr_ids.second, c2);
        setup_constraint(data2, c2, friction_ratio.y);
    }

    void            setup_friction_cone(ConstraintIds constr_ids, const Constraint& c1, const Constraint& c2, vec2 friction_ratio, ConstraintId normal_constr_id)
    {
        auto setup_constraint = [this, normal_constr_id](ConstraintData& data, const Constraint& c, float friction_ratio) {
            data.cfm_inv_dt = c.cfm * m_inv_dt;
            data.bg_error = 0.f;

            data.friction_ratio = friction_ratio;
            data.normal_constr_idx = normal_constr_id.index();
        };

        auto& data1 = basic_setup(constr_ids.first, c1);
        setup_constraint(data1, c1, friction_ratio.x);

        auto& data2 = basic_setup(constr_ids.second, c2);
        setup_constraint(data2, c2, friction_ratio.y);
    }

    /*
    ConstraintId    add_constraint(const Constraint& c);
    ConstraintId    join_friction_1d(const Constraint& c, float friction_ratio, ConstraintId normal_constr_id);
    ConstraintIds   join_friction_2d(const Constraint& c1, const Constraint& c2, vec2 friction_ratio, ConstraintId normal_constr_id);
    ConstraintIds   join_friction_cone(const Constraint& c1, const Constraint& c2, vec2 friction_ratio, ConstraintId normal_constr_id);
*/

    float           get_lambda(ConstraintId constr_id) const;
    void            clear();

    void            setup_solve_executor(TaskExecutor& executor, Fence fence);

protected:
    struct alignas(16) ConstraintData {
        vec3    jacobian11;
        vec3    jacobian12;
        vec3    jacobian21;
        vec3    jacobian22;

        vec3    inv_m_j11;
        vec3    inv_m_j12;
        vec3    inv_m_j21;
        vec3    inv_m_j22;

        float   min_bound;
        float   max_bound;

        int     body1_idx = -1;
        int     body2_idx = -1;

        float   cfm_inv_dt;
        float   bg_error;

        float   rhs;
        float   inv_diag;

        float   friction_ratio;
        int     normal_constr_idx = -1;
    };

    struct alignas(16) BodyData {
        vec3        inv_m_f1;
        vec3        inv_m_f2;
        float       _pad[2] = {};
    };

    struct BodyExtraData {
        vec3        v_delta1;
        vec3        v_delta2;
        RigidBody*  body = nullptr;
    };

    struct GroupData {
        Vector<ConstraintData> constraints;
        Vector<float> lambda;
        int size = 0;
    };

    struct TaskContext {

    };

    ConstraintData& basic_setup(ConstraintId constr_id, const Constraint& c)
    {
        SL_ASSERT(c.body1->in_solver_index() != -1);

        auto& group = m_groups[(int)constr_id.group()];
        auto& data = group.constraints[constr_id.index()];

        data.body1_idx = c.body1->in_solver_index();
        data.jacobian11 = c.jacobian1[0];
        data.jacobian12 = c.jacobian1[1];

        if (c.body2) {
            SL_ASSERT(c.body2->in_solver_index() != -1);

            data.body2_idx = c.body2->in_solver_index();
            data.jacobian21 = c.jacobian2[0];
            data.jacobian22 = c.jacobian2[1];
        } else {
            data.body2_idx = -1;
            data.jacobian21.set_zero();
            data.jacobian22.set_zero();
        }

        data.normal_constr_idx = -1;

        group.lambda[constr_id.index()] = c.init_lambda;

        return data;
    }

    void prepare_data(TaskExecutor& executor, Fence fence);

    //auto create_constraint(const Constraint& c, ConstraintGroup group) -> std::pair<ConstraintId, ConstraintData*>;
    void apply_impulses();

    template<bool UseSIMD>
    void solve_iterations();

    float   m_dt = 1.f;
    float   m_inv_dt = 1.f;
    Config  m_config;

    Vector<BodyData> m_bodies;
    Vector<BodyExtraData> m_bodies_extra;
    //Array<std::unique_ptr<GroupData>, (int)ConstraintGroup::Count> m_groups;
    Array<GroupData, (int)ConstraintGroup::Count> m_groups;

    Vector<TaskContext> m_task_ctx;

    friend struct ConstraintHelper;
};

inline void ConstraintSolver::set_time_interval(float value) {
    m_dt = value;
    m_inv_dt = 1.f / value;
}

inline Constraint Constraint::bilateral(RigidBody* body1, RigidBody* body2, const ConstraintGeom& geom) {
    return generic(body1, body2, geom, 0.f, -FLOAT_MAX, FLOAT_MAX);
}

inline Constraint Constraint::bilateral(RigidBody* body1, const ConstraintGeom& geom) {
    return generic(body1, nullptr, geom, 0.f, -FLOAT_MAX, FLOAT_MAX);
}

inline Constraint Constraint::unilateral(RigidBody* body1, RigidBody* body2, const ConstraintGeom& geom) {
    return generic(body1, body2, geom, 0.f, 0.f, FLOAT_MAX);
}

inline Constraint Constraint::unilateral(RigidBody* body1, const ConstraintGeom& geom) {
    return generic(body1, nullptr, geom, 0.f, 0.f, FLOAT_MAX);
}

inline Constraint Constraint::stabilized_bilateral(RigidBody* body1, RigidBody* body2, const ConstraintGeom& geom) {
    return generic(body1, body2, geom, geom.pos_error(), -FLOAT_MAX, FLOAT_MAX);
}

inline Constraint Constraint::stabilized_bilateral(RigidBody* body1, const ConstraintGeom& geom) {
    return generic(body1, nullptr, geom, geom.pos_error(), -FLOAT_MAX, FLOAT_MAX);
}

inline Constraint Constraint::stabilized_unilateral(RigidBody* body1, RigidBody* body2, const ConstraintGeom& geom) {
    return generic(body1, body2, geom, geom.pos_error(), 0.f, FLOAT_MAX);
}

inline Constraint Constraint::stabilized_unilateral(RigidBody* body1, const ConstraintGeom& geom) {
    return generic(body1, nullptr, geom, geom.pos_error(), 0.f, FLOAT_MAX);
}

inline float ConstraintSolver::get_lambda(ConstraintId constr_id) const {
    return m_groups[(int)constr_id.group()].lambda[constr_id.index()];
}

} // slope
