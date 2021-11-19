#include "slope/simulation/constraint_solver.hpp"
#include "slope/debug/assert.hpp"

namespace slope {

void ConstraintSolver::register_body(RigidBody* body) {
    body->set_in_solver_index(static_cast<int>(m_bodies.size()));
    m_bodies.push_back({body});
}

std::pair<ConstraintId, ConstraintSolver::ConstraintData*> ConstraintSolver::create_constraint(Constraint& c, int group) {
    if (c.body1->in_solver_index() == -1)
        register_body(c.body1);

    auto& data = m_constraints[group].emplace_back();

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

    return { { group, static_cast<int>(m_constraints[group].size() - 1) }, &data };
}

ConstraintId ConstraintSolver::add_constraint(Constraint& c) {
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

ConstraintId ConstraintSolver::join_friction(Constraint& c, float friction_ratio, ConstraintId normal_constr_id) {
    auto [constr_id, data] = create_constraint(c, ConstraintGroup::Friction);

    data->cfm_inv_dt = c.cfm * m_inv_dt;
    data->bg_error = 0.f;

    data->friction_ratio = friction_ratio;
    data->normal_constr_idx = normal_constr_id.index();

    return constr_id;
}

void ConstraintSolver::clear() {
    for (auto& body : m_bodies)
        body.body->set_in_solver_index(-1);

    m_bodies.clear();

    for (auto& container : m_constraints)
        container.clear();
}

void ConstraintSolver::solve_constraint(ConstraintData& c, float min_bound, float max_bound) {
    if (c.body2_idx >= 0) {
        auto& b1 = m_bodies[c.body1_idx];
        auto& b2 = m_bodies[c.body2_idx];

        float cur_lambda = c.lambda;

        float dot = c.jacobian1[0].dot(b1.inv_m_f[0]) + c.jacobian1[1].dot(b1.inv_m_f[1]);
        dot += c.jacobian2[0].dot(b2.inv_m_f[0]) + c.jacobian2[1].dot(b2.inv_m_f[1]);
        dot += cur_lambda * c.cfm_inv_dt;

        float delta = (c.rhs - dot) * c.inv_diag;
        c.lambda = clamp(min_bound, cur_lambda + delta, max_bound);
        delta = c.lambda- cur_lambda;

        b1.inv_m_f[0] += c.inv_m_j1[0] * delta;
        b1.inv_m_f[1] += c.inv_m_j1[1] * delta;
        b2.inv_m_f[0] += c.inv_m_j2[0] * delta;
        b2.inv_m_f[1] += c.inv_m_j2[1] * delta;

    } else {
        auto& b1 = m_bodies[c.body1_idx];

        float cur_lambda = c.lambda;

        float dot = c.jacobian1[0].dot(b1.inv_m_f[0]) + c.jacobian1[1].dot(b1.inv_m_f[1]);
        dot += cur_lambda * c.cfm_inv_dt;

        float delta = (c.rhs - dot) * c.inv_diag;
        c.lambda = clamp(min_bound, cur_lambda + delta, max_bound);
        delta = c.lambda- cur_lambda;

        b1.inv_m_f[0] += c.inv_m_j1[0] * delta;
        b1.inv_m_f[1] += c.inv_m_j1[1] * delta;
    }
}

void ConstraintSolver::prepare_data() {
    static constexpr float INV_DIAG_EPSILON = 1e-8f;

    // V_delta = V / dt + F * M_inv
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
            c.inv_diag = rcp * rcp > INV_DIAG_EPSILON ? m_sor / rcp : 0.f;
        }
    }
}

void ConstraintSolver::solve() {
    if (m_constraints.empty())
        return;

    prepare_data();

    // Projected Gauss-Seidel
    for(uint32_t iter = 0; iter < m_iteration_count; ++iter) {
        for (auto& c : m_constraints[ConstraintGroup::Normal]) {
            solve_constraint(c, c.min_bound, c.max_bound);
        }

        for (auto& c : m_constraints[ConstraintGroup::Friction]) {
            float normal_lambda = m_constraints[ConstraintGroup::Normal][c.normal_constr_idx].lambda;
            float bound = c.friction_ratio * fabsf(normal_lambda);
            solve_constraint(c, -bound, bound);
        }
    }

    apply_impulses();
}

void ConstraintSolver::apply_impulses() {
    for (auto& b : m_bodies) {
        auto* body = b.body;
        body->set_velocity(body->velocity() + b.inv_m_f[0] * m_dt);
        body->set_ang_velocity(body->ang_velocity() + b.inv_m_f[1] * m_dt);
    }
}

} // slope
