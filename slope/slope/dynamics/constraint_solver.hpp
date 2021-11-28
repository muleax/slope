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
    Normal = 0,
    Friction = 1,
    Count
};

class ConstraintId {
public:
    ConstraintId() = default;
    ConstraintId(int raw) : m_raw(raw) {}
    ConstraintId(ConstraintGroup group, int index) : m_raw((int)group | index << 1) {}

    bool            is_valid() const { return m_raw >= 0; }
    ConstraintGroup group() const { return ConstraintGroup(m_raw & 1); }
    int             index() const { return m_raw >> 1; }
    int             raw() const { return m_raw; }

    operator int() const { return m_raw; }

private:
    int m_raw = -1;
};

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

class ConstraintSolver
{
public:
    struct Config {
        // Successive over-relaxation
        float       sor = 1.f;
        int         iteration_count = 10;
    };

    virtual ~ConstraintSolver() = default;

    Config&         config() { return m_config; }
    const Config&   config() const { return m_config; }

    void            set_time_interval(float value);
    float           time_interval() const { return m_dt; }

    ConstraintId    add_constraint(Constraint& c);
    ConstraintId    join_friction(Constraint& c, float friction_ratio, ConstraintId normal_constr_id);

    void            solve();
    void            clear();

    float           get_lambda(ConstraintId constr_id) const;

    float           max_error() const;
    float           avg_error() const;

protected:
    struct ConstraintData {
        int     body1_idx = -1;
        int     body2_idx = -1;

        Vec3    jacobian1[2];
        Vec3    jacobian2[2];

        Vec3    inv_m_j1[2];
        Vec3    inv_m_j2[2];

        float   min_bound;
        float   max_bound;
        float   lambda;
        float   delta_lambda;

        float   bg_error;
        float   cfm_inv_dt;

        float   rhs;
        float   inv_diag;

        float   friction_ratio;
        int     normal_constr_idx = -1;
    };

    struct BodyData {
        RigidBody*  body;
        Vec3        v_delta[2];
        Vec3        inv_m_f[2];
    };

    void            register_body(RigidBody* body);
    auto            create_constraint(Constraint& c, ConstraintGroup group) -> std::pair<ConstraintId, ConstraintData*>;
    void            prepare_data();
    void            apply_impulses();

    virtual void    solve_impl() = 0;

    float   m_dt = 1.f;
    float   m_inv_dt = 1.f;
    Config  m_config;

    Vector<BodyData> m_bodies;
    Array<Vector<ConstraintData>, (int)ConstraintGroup::Count> m_constraints;
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
    return m_constraints[(int)constr_id.group()][constr_id.index()].lambda;
}

} // slope
