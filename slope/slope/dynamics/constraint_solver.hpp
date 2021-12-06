#pragma once
#include "slope/dynamics/rigid_body.hpp"
#include "slope/containers/vector.hpp"
#include "slope/containers/array.hpp"

namespace slope {

struct ConstraintGeom {
    Vec3 p1;
    Vec3 p2;
    Vec3 axis;

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

    Vec3        jacobian1[2];
    Vec3        jacobian2[2];

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

    virtual ~ConstraintSolver() = default;

    Config&         config() { return m_config; }
    const Config&   config() const { return m_config; }

    void            set_time_interval(float value);
    float           time_interval() const { return m_dt; }

    ConstraintId    add_constraint(const Constraint& c);
    ConstraintId    join_friction_1d(const Constraint& c, float friction_ratio, ConstraintId normal_constr_id);
    ConstraintIds   join_friction_2d(const Constraint& c1, const Constraint& c2, Vec2 friction_ratio, ConstraintId normal_constr_id);
    ConstraintIds   join_friction_cone(const Constraint& c1, const Constraint& c2, Vec2 friction_ratio, ConstraintId normal_constr_id);

    virtual void    solve();
    void            clear();

    float           get_lambda(ConstraintId constr_id) const;

protected:
    struct alignas(16) ConstraintData {
        Vec3    jacobian11;
        Vec3    jacobian12;
        Vec3    jacobian21;
        Vec3    jacobian22;

        Vec3    inv_m_j11;
        Vec3    inv_m_j12;
        Vec3    inv_m_j21;
        Vec3    inv_m_j22;

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
        Vec3        inv_m_f1;
        Vec3        inv_m_f2;
        float       _pad[2] = {};
    };

    struct BodyExtraData {
        Vec3        v_delta1;
        Vec3        v_delta2;
        RigidBody*  body = nullptr;
    };

    struct GroupData {
        Vector<ConstraintData> constraints;
        Vector<float> lambda;
    };

    void register_body(RigidBody* body);
    auto create_constraint(const Constraint& c, ConstraintGroup group) -> std::pair<ConstraintId, ConstraintData*>;
    void prepare_data();
    void apply_impulses();

    template<bool UseSIMD>
    void solve_iterations();

    float   m_dt = 1.f;
    float   m_inv_dt = 1.f;
    Config  m_config;

    Vector<BodyData> m_bodies;
    Vector<BodyExtraData> m_bodies_extra;
    Array<GroupData, (int)ConstraintGroup::Count> m_groups;

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
