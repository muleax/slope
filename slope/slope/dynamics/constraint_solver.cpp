#include "slope/dynamics/constraint_solver.hpp"
#include "slope/debug/assert.hpp"
#include "slope/debug/log.hpp"

namespace slope {

Constraint Constraint::generic(
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

void ConstraintSolver::register_body(RigidBody* body) {
    body->set_in_solver_index(static_cast<int>(m_bodies.size()));
    m_bodies.push_back({body});
}

std::pair<ConstraintId, ConstraintSolver::ConstraintData*> ConstraintSolver::create_constraint(const Constraint& c, ConstraintGroup group) {
    if (c.body1->in_solver_index() == -1)
        register_body(c.body1);

    auto& data = m_constraints[(int)group].emplace_back();

    data.body1_idx = c.body1->in_solver_index();
    data.jacobian1[0] = c.jacobian1[0];
    data.jacobian1[1] = c.jacobian1[1];

    if (c.body2) {
        if (c.body2->in_solver_index() == -1)
            register_body(c.body2);

        data.body2_idx = c.body2->in_solver_index();
        data.jacobian2[0] = c.jacobian2[0];
        data.jacobian2[1] = c.jacobian2[1];
    }

    data.lambda = c.init_lambda;

    return { { group, static_cast<int>(m_constraints[(int)group].size() - 1) }, &data };
}

ConstraintId ConstraintSolver::add_constraint(const Constraint& c) {
    auto [constr_id, data] = create_constraint(c, ConstraintGroup::Normal);

    data->min_bound = c.min_bound;
    data->max_bound = c.max_bound;
    data->cfm_inv_dt = c.cfm * m_inv_dt;

    // TODO: reconsider
    if (c.min_bound == 0.f) {
        // unilateral case
        if (c.pos_error > c.unilateral_penetration)
            data->bg_error = (c.pos_error - c.unilateral_penetration) * c.erp;
        else
            data->bg_error = c.pos_error - c.unilateral_penetration;
    } else {
        // bilateral case
        data->bg_error = c.pos_error * c.erp;
    }

    return constr_id;
}

ConstraintId ConstraintSolver::join_friction(const Constraint& c, float friction_ratio, ConstraintId normal_constr_id) {
    auto [constr_id, data] = create_constraint(c, ConstraintGroup::Friction);

    data->cfm_inv_dt = c.cfm * m_inv_dt;
    data->bg_error = 0.f;

    data->friction_ratio = friction_ratio;
    data->normal_constr_idx = normal_constr_id.index();

    return constr_id;
}

ConstraintIds ConstraintSolver::join_cone_friction(const Constraint& c1, const Constraint& c2, Vec2 friction_ratio, ConstraintId normal_constr_id)
{
    auto [constr_id1, data1] = create_constraint(c1, ConstraintGroup::ConeFriction);
    auto [constr_id2, data2] = create_constraint(c2, ConstraintGroup::ConeFriction);

    auto setup_constraint = [this, normal_constr_id](ConstraintData* data, const Constraint& c, float friction_ratio) {
        data->cfm_inv_dt = c.cfm * m_inv_dt;
        data->bg_error = 0.f;

        data->friction_ratio = friction_ratio;
        data->normal_constr_idx = normal_constr_id.index();
    };

    setup_constraint(data1, c1, friction_ratio.x);
    setup_constraint(data2, c2, friction_ratio.y);

    return {constr_id1, constr_id2};
}

void ConstraintSolver::clear() {
    for (auto& body : m_bodies)
        body.body->set_in_solver_index(-1);

    m_bodies.clear();

    for (auto& container : m_constraints)
        container.clear();
}

void ConstraintSolver::prepare_data() {
    static constexpr float INV_DIAG_EPSILON = 1e-8f;

    // V_delta = V / dt + M_inv * F
    for (auto& b : m_bodies) {
        auto* body = b.body;
        b.v_delta[0] = body->velocity() * m_inv_dt + body->force() * body->inv_mass();
        b.v_delta[1] = body->ang_velocity() * m_inv_dt + body->inv_inertia().apply_normal(body->torque());

        b.inv_m_f[0].set_zero();
        b.inv_m_f[1].set_zero();
    }

    // J * M_inv * J_t * lambda = pos_error / dt^2 - J * V_delta
    // inv_diag = 1 / (cfm / dt + J * M_inv * J_t)
    // M_inv * force = M_inv * J_t * lambda
    for (auto& container : m_constraints) {
        for (auto& c: container) {
            auto& b1 = m_bodies[c.body1_idx];

            float j_v_delta = c.jacobian1[0].dot(b1.v_delta[0]) + c.jacobian1[1].dot(b1.v_delta[1]);

            c.inv_m_j1[0] = c.jacobian1[0] * b1.body->inv_mass();
            c.inv_m_j1[1] = b1.body->inv_inertia().apply_normal(c.jacobian1[1]);

            b1.inv_m_f[0] += c.inv_m_j1[0] * c.lambda;
            b1.inv_m_f[1] += c.inv_m_j1[1] * c.lambda;

            float j_inv_m_j = c.jacobian1[0].dot(c.inv_m_j1[0]) + c.jacobian1[1].dot(c.inv_m_j1[1]);

            if (c.body2_idx >= 0) {
                auto& b2 = m_bodies[c.body2_idx];
                j_v_delta += c.jacobian2[0].dot(b2.v_delta[0]) + c.jacobian2[1].dot(b2.v_delta[1]);

                c.inv_m_j2[0] = c.jacobian2[0] * b2.body->inv_mass();
                c.inv_m_j2[1] = b2.body->inv_inertia().apply_normal(c.jacobian2[1]);

                b2.inv_m_f[0] += c.inv_m_j2[0] * c.lambda;
                b2.inv_m_f[1] += c.inv_m_j2[1] * c.lambda;

                j_inv_m_j += c.jacobian2[0].dot(c.inv_m_j2[0]) + c.jacobian2[1].dot(c.inv_m_j2[1]);
            }

            c.rhs = m_inv_dt * m_inv_dt * c.bg_error - j_v_delta;

            float rcp = c.cfm_inv_dt + j_inv_m_j;
            c.inv_diag = rcp * rcp > INV_DIAG_EPSILON ? m_config.sor / rcp : 0.f;
        }
    }
}

inline float ConstraintSolver::solve_constraint_lambda(ConstraintData& c) {
    float delta;
    float dot = c.cfm_inv_dt;

    if (c.body2_idx >= 0) {
        auto& b1 = m_bodies[c.body1_idx];
        auto& b2 = m_bodies[c.body2_idx];
        dot += c.jacobian1[0].dot(b1.inv_m_f[0]) + c.jacobian1[1].dot(b1.inv_m_f[1]);
        dot += c.jacobian2[0].dot(b2.inv_m_f[0]) + c.jacobian2[1].dot(b2.inv_m_f[1]);

        delta = (c.rhs - dot) * c.inv_diag;

    } else {
        auto& b1 = m_bodies[c.body1_idx];
        dot += c.jacobian1[0].dot(b1.inv_m_f[0]) + c.jacobian1[1].dot(b1.inv_m_f[1]);

        delta = (c.rhs - dot) * c.inv_diag;
    }

    return c.lambda + delta;
}

inline void ConstraintSolver::apply_constraint_lambda(ConstraintData& c, float lambda) {
    if (c.body2_idx >= 0) {
        auto& b1 = m_bodies[c.body1_idx];
        auto& b2 = m_bodies[c.body2_idx];

        float delta = lambda - c.lambda;
        c.lambda = lambda;
        c.delta_lambda = delta;

        b1.inv_m_f[0] += c.inv_m_j1[0] * delta;
        b1.inv_m_f[1] += c.inv_m_j1[1] * delta;
        b2.inv_m_f[0] += c.inv_m_j2[0] * delta;
        b2.inv_m_f[1] += c.inv_m_j2[1] * delta;

    } else {
        auto& b1 = m_bodies[c.body1_idx];

        float delta = lambda - c.lambda;
        c.lambda = lambda;
        c.delta_lambda = delta;

        b1.inv_m_f[0] += c.inv_m_j1[0] * delta;
        b1.inv_m_f[1] += c.inv_m_j1[1] * delta;
    }
}

inline void ConstraintSolver::solve_constraint(ConstraintData& c, float min_bound, float max_bound)
{
    float cur_lambda = c.lambda;
    float dot = cur_lambda * c.cfm_inv_dt;

    if (c.body2_idx >= 0) {
        auto& b1 = m_bodies[c.body1_idx];
        auto& b2 = m_bodies[c.body2_idx];

        dot += c.jacobian1[0].dot(b1.inv_m_f[0]) + c.jacobian1[1].dot(b1.inv_m_f[1]);
        dot += c.jacobian2[0].dot(b2.inv_m_f[0]) + c.jacobian2[1].dot(b2.inv_m_f[1]);

        float delta = (c.rhs - dot) * c.inv_diag;
        c.lambda = clamp(min_bound, cur_lambda + delta, max_bound);
        delta = c.lambda - cur_lambda;
        c.delta_lambda = delta;

        b1.inv_m_f[0] += c.inv_m_j1[0] * delta;
        b1.inv_m_f[1] += c.inv_m_j1[1] * delta;
        b2.inv_m_f[0] += c.inv_m_j2[0] * delta;
        b2.inv_m_f[1] += c.inv_m_j2[1] * delta;

    } else {
        auto& b1 = m_bodies[c.body1_idx];

        dot += c.jacobian1[0].dot(b1.inv_m_f[0]) + c.jacobian1[1].dot(b1.inv_m_f[1]);

        float delta = (c.rhs - dot) * c.inv_diag;
        c.lambda = clamp(min_bound, cur_lambda + delta, max_bound);
        delta = c.lambda - cur_lambda;
        c.delta_lambda = delta;

        b1.inv_m_f[0] += c.inv_m_j1[0] * delta;
        b1.inv_m_f[1] += c.inv_m_j1[1] * delta;
    }
}

inline void ConstraintSolver::solve_cone_friction(ConstraintData& c1, ConstraintData& c2)
{
    float lambda1 = 0.f;
    float lambda2 = 0.f;

    float normal_lambda = fabsf(m_constraints[(int)ConstraintGroup::Normal][c1.normal_constr_idx].lambda);
    if (normal_lambda > 0.f) {
        float bound1 = c1.friction_ratio * normal_lambda;
        float bound2 = c2.friction_ratio * normal_lambda;

        lambda1 = solve_constraint_lambda(c1);
        lambda2 = solve_constraint_lambda(c2);

        double divisor = ((double)lambda1 * lambda1) * ((double)bound2 * bound2) + ((double)lambda2 * lambda2) * ((double)bound1 * bound1);
        double product = (double)bound1 * bound2;

        if (divisor > 1e-8 && divisor > product * product) {
            // scale down and preserve angle
            double t = product / sqrt(divisor);
            lambda1 = float(t * lambda1);
            lambda2 = float(t * lambda2);

            // float angle = std::atan2f(lambda1, lambda2);
            // lambda1 = bound1 * std::sinf(angle);
            // lambda2 = bound2 * std::cosf(angle);

        } else {
            lambda1 = clamp(-bound1, lambda1, bound1);
            lambda2 = clamp(-bound2, lambda2, bound2);
        }
    }

    apply_constraint_lambda(c1, lambda1);
    apply_constraint_lambda(c2, lambda2);
}

void ConstraintSolver::solve_iterations()
{
    for(uint32_t iter = 0; iter < m_config.iteration_count; ++iter) {

        for (auto& c: m_constraints[(int) ConstraintGroup::Normal]) {
            solve_constraint(c, c.min_bound, c.max_bound);
        }

        for (auto& c: m_constraints[(int) ConstraintGroup::Friction]) {
            float normal_lambda = fabsf(m_constraints[(int) ConstraintGroup::Normal][c.normal_constr_idx].lambda);
            float bound = c.friction_ratio * normal_lambda;
            solve_constraint(c, -bound, bound);
        }

        auto& cone_friction_constrs = m_constraints[(int) ConstraintGroup::ConeFriction];
        for (auto it = cone_friction_constrs.begin(); it != cone_friction_constrs.end();) {
            auto& c1 = *(it++);
            auto& c2 = *(it++);
            solve_cone_friction(c1, c2);
        }
    }
}

void ConstraintSolver::solve() {
    if (!m_constraints.empty()) {
        prepare_data();
        solve_iterations();
        apply_impulses();
    }
}

void ConstraintSolver::apply_impulses() {
    for (auto& b : m_bodies) {
        auto* body = b.body;
        body->set_velocity(body->velocity() + b.inv_m_f[0] * m_dt);
        body->set_ang_velocity(body->ang_velocity() + b.inv_m_f[1] * m_dt);
    }
}

float ConstraintSolver::max_error() const {
    float err = 0.f;
    for (auto& group : m_constraints)
        for (auto& c : group)
            err = std::fmax(err, fabs(c.delta_lambda));

    return err;
}

float ConstraintSolver::avg_error() const {
    float err = 0.f;
    size_t constraint_count = 0;
    for (auto& group : m_constraints) {
        constraint_count += group.size();
        for (auto& c: group)
            err += fabs(c.delta_lambda);
    }

    if (constraint_count == 0)
        return 0.f;

    return err / static_cast<float>(constraint_count);
}

} // slope
