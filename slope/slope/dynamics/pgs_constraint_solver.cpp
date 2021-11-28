#include "slope/dynamics/pgs_constraint_solver.hpp"

namespace slope {

void PGSConstraintSolver::solve_constraint(ConstraintData& c, float min_bound, float max_bound)
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

void PGSConstraintSolver::solve_impl()
{
    for(uint32_t iter = 0; iter < m_config.iteration_count; ++iter) {

        for (auto& c : m_constraints[(int)ConstraintGroup::Normal]) {
            solve_constraint(c, c.min_bound, c.max_bound);
        }

        for (auto& c : m_constraints[(int)ConstraintGroup::Friction]) {
            float normal_lambda = m_constraints[(int)ConstraintGroup::Normal][c.normal_constr_idx].lambda;
            float bound = c.friction_ratio * fabsf(normal_lambda);
            solve_constraint(c, -bound, bound);
        }
    }
}

} // slope
