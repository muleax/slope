#pragma once
#include "slope/simulation/rigid_body.hpp"
#include "slope/containers/vector.hpp"
#include "slope/containers/array.hpp"

namespace slope {

struct ConstraintGeom {
    Vec3 p1;
    Vec3 p2;
    Vec3 axis;

    float pos_error() const { return axis.dot(p1 - p2); }
};

struct ConstraintGroup {
    enum {
        Normal = 0,
        Friction = 1
    };
};

class ConstraintId {
public:
    ConstraintId() = default;
    ConstraintId(int raw) : m_raw(raw) {}
    ConstraintId(int group, int index) : m_raw(group | index << 1) {}

    bool is_valid() const { return m_raw >= 0; }
    int group() const { return m_raw & 1; }
    int index() const { return m_raw >> 1; }
    int raw() const { return m_raw; }

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
    void            set_time_interval(float value);
    float           time_interval() const { return m_dt; }

    // Successive over-relaxation
    void            set_sor(float value) { m_sor = value; }
    float           sor() const { return m_sor; }

    void            set_iteration_count(uint32_t value) { m_iteration_count = value; }
    uint32_t        iteration_count() const { return m_iteration_count; }

    ConstraintId    add_constraint(Constraint& c);
    ConstraintId    join_friction(Constraint& c, float friction_ratio, ConstraintId normal_constr_id);

    void            solve();
    void            clear();

    float           get_lambda(ConstraintId constr_id) const;

private:
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

    void register_body(RigidBody* body);
    auto create_constraint(Constraint& c, int group) -> std::pair<ConstraintId, ConstraintData*>;
    void prepare_data();
    void solve_constraint(ConstraintData& c, float min_bound, float max_bound);
    void apply_impulses();

    float       m_dt = 1.f;
    float       m_inv_dt = 1.f;
    float       m_sor = 1.f;
    uint32_t    m_iteration_count = 10;

    Vector<BodyData>                    m_bodies;
    Array<Vector<ConstraintData>, 2>    m_constraints;
};

inline void ConstraintSolver::set_time_interval(float value) {
    m_dt = value;
    m_inv_dt = 1.f / value;
}

inline Constraint Constraint::generic(
        RigidBody* body1, RigidBody* body2, const ConstraintGeom& geom,
        float pos_error, float min_bound, float max_bound) {

    Constraint c;
    c.body1 = body1;

    Vec3 r1 = geom.p1 - body1->transform().translation();
    c.jacobian1[0] = -geom.axis;
    c.jacobian1[1] = -r1.cross(geom.axis);

    if (body2) {
        c.body2 = body2;

        Vec3 r2 = geom.p2 - body2->transform().translation();
        c.jacobian2[0] = geom.axis;
        c.jacobian2[1] = r2.cross(geom.axis);
    }

    c.pos_error = pos_error;
    c.min_bound = min_bound;
    c.max_bound = max_bound;

    return c;
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
    return m_constraints[constr_id.group()][constr_id.index()].lambda;
}

} // slope
