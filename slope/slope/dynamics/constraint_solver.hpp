#pragma once
#include "slope/core/vector.hpp"
#include "slope/core/array.hpp"
#include "slope/core/config_holder.hpp"
#include "slope/dynamics/rigid_body.hpp"

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
    explicit ConstraintId(int raw) : m_raw(raw) {}
    ConstraintId(ConstraintGroup group, int index) : m_raw((int)group | index << 2) {}

    bool            is_valid() const { return m_raw >= 0; }
    ConstraintGroup group() const { return ConstraintGroup(m_raw & 3); }
    int             index() const { return m_raw >> 2; }
    int             raw() const { return m_raw; }

    ConstraintId    operator+(int delta) const { return {group(), index() + delta}; }
    ConstraintId    operator-(int delta) const { return {group(), index() - delta}; }
    void            operator+=(int delta) { *this = *this + delta; }
    void            operator-=(int delta) { *this = *this - delta; }
    void            operator++() { *this += 1; }
    void            operator--() { *this -= 1; }

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
    float       restitution = 0.f;
};

struct ConstraintSolverConfig {
    bool        use_simd = true;
    // Successive over-relaxation
    float       sor = 1.f;
    int         iteration_count = 10;
    float       time_interval = 1.f / 60.f;
};

// Projected Gauss-Seidel constraint solver
class ConstraintSolver : public ConfigHolder<ConstraintSolverConfig>
{
public:
    explicit ConstraintSolver(int concurrency = 1);

    void            set_concurrency(int concurrency);
    int             concurrency() const;

    void            register_body(RigidBody* body);

    ConstraintId    allocate(ConstraintGroup group, int count);

    void            setup_constraint(ConstraintId id, const Constraint& c);
    void            setup_friction_1d(ConstraintId id, const Constraint& c, float ratio, ConstraintId normal_id);
    void            setup_friction_2d(ConstraintIds ids, const Constraint& c1, const Constraint& c2, vec2 ratio, ConstraintId normal_id);
    void            setup_friction_cone(ConstraintIds ids, const Constraint& c1, const Constraint& c2, vec2 ratio, ConstraintId normal_id);

    ConstraintId    add_constraint(const Constraint& c);
    ConstraintId    add_friction_1d(const Constraint& c, float ratio, ConstraintId normal_id);
    ConstraintIds   add_friction_2d(const Constraint& c1, const Constraint& c2, vec2 ratio, ConstraintId normal_id);
    ConstraintIds   add_friction_cone(const Constraint& c1, const Constraint& c2, vec2 ratio, ConstraintId normal_id);

    int             get_constraint_count(ConstraintGroup group) const;
    float           get_lambda(ConstraintId constr_id) const;
    const vec3&     get_linear_axis(ConstraintId constr_id) const;
    const vec3&     get_angular_axis(ConstraintId constr_id) const;
    void            clear();

    void            solve_pass0();
    void            solve_pass1(int worker_id);
    void            solve_pass2();

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
        float   restitution;

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
        Vector<ConstraintData>  constraints;
        Vector<float>           lambda;
        int                     size = 0;
    };

    struct WorkerContext {
        Vector<BodyData> bodies;
    };

    using Groups = Array<GroupData, (int)ConstraintGroup::Count>;

    void            on_config_update(const ConstraintSolverConfig& prev_config) override;
    void            set_time_interval(float time_interval);
    ConstraintData& basic_setup(ConstraintId id, const Constraint& c);

    void apply_impulses();

    template<bool UseSIMD>
    void solve_iterations();

    float                   m_dt = 1.f;
    float                   m_inv_dt = 1.f;

    Vector<BodyData>        m_bodies;
    Vector<BodyExtraData>   m_bodies_extra;
    Groups                  m_groups;
    Vector<WorkerContext>   m_worker_ctx;

    friend struct ConstraintHelper;
};

inline ConstraintId ConstraintSolver::add_constraint(const Constraint& c)
{
    ConstraintId id = allocate(ConstraintGroup::General, 1);
    setup_constraint(id, c);
    return id;
}

inline ConstraintId ConstraintSolver::add_friction_1d(const Constraint& c, float ratio, ConstraintId normal_id)
{
    ConstraintId id = allocate(ConstraintGroup::Friction1D, 1);
    setup_friction_1d(id, c, ratio, normal_id);
    return id;
}

inline ConstraintIds ConstraintSolver::add_friction_2d(const Constraint& c1, const Constraint& c2, vec2 ratio, ConstraintId normal_id)
{
    ConstraintId first = allocate(ConstraintGroup::Friction2D, 2);
    ConstraintIds ids = {first, first + 1};
    setup_friction_2d(ids, c1, c2, ratio, normal_id);
    return ids;
}

inline ConstraintIds ConstraintSolver::add_friction_cone(const Constraint& c1, const Constraint& c2, vec2 ratio, ConstraintId normal_id)
{
    ConstraintId first = allocate(ConstraintGroup::FrictionCone, 2);
    ConstraintIds ids = {first, first + 1};
    setup_friction_cone(ids, c1, c2, ratio, normal_id);
    return ids;
}

inline Constraint Constraint::bilateral(RigidBody* body1, RigidBody* body2, const ConstraintGeom& geom)
{
    return generic(body1, body2, geom, 0.f, -FLOAT_MAX, FLOAT_MAX);
}

inline Constraint Constraint::bilateral(RigidBody* body1, const ConstraintGeom& geom)
{
    return generic(body1, nullptr, geom, 0.f, -FLOAT_MAX, FLOAT_MAX);
}

inline Constraint Constraint::unilateral(RigidBody* body1, RigidBody* body2, const ConstraintGeom& geom)
{
    return generic(body1, body2, geom, 0.f, 0.f, FLOAT_MAX);
}

inline Constraint Constraint::unilateral(RigidBody* body1, const ConstraintGeom& geom)
{
    return generic(body1, nullptr, geom, 0.f, 0.f, FLOAT_MAX);
}

inline Constraint Constraint::stabilized_bilateral(RigidBody* body1, RigidBody* body2, const ConstraintGeom& geom)
{
    return generic(body1, body2, geom, geom.pos_error(), -FLOAT_MAX, FLOAT_MAX);
}

inline Constraint Constraint::stabilized_bilateral(RigidBody* body1, const ConstraintGeom& geom)
{
    return generic(body1, nullptr, geom, geom.pos_error(), -FLOAT_MAX, FLOAT_MAX);
}

inline Constraint Constraint::stabilized_unilateral(RigidBody* body1, RigidBody* body2, const ConstraintGeom& geom)
{
    return generic(body1, body2, geom, geom.pos_error(), 0.f, FLOAT_MAX);
    return generic(body1, body2, geom, geom.pos_error(), 0.f, FLOAT_MAX);
}

inline Constraint Constraint::stabilized_unilateral(RigidBody* body1, const ConstraintGeom& geom)
{
    return generic(body1, nullptr, geom, geom.pos_error(), 0.f, FLOAT_MAX);
}

inline int ConstraintSolver::get_constraint_count(ConstraintGroup group) const
{
    return m_groups[(int)group].size;
}

inline float ConstraintSolver::get_lambda(ConstraintId constr_id) const
{
    return m_groups[(int)constr_id.group()].lambda[constr_id.index()];
}

inline const vec3& ConstraintSolver::get_linear_axis(ConstraintId constr_id) const
{
    return m_groups[(int)constr_id.group()].constraints[constr_id.index()].jacobian11;
}

inline const vec3& ConstraintSolver::get_angular_axis(ConstraintId constr_id) const
{
    return m_groups[(int)constr_id.group()].constraints[constr_id.index()].jacobian12;
}

} // slope
