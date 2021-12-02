#include "slope/dynamics/pj_constraint_solver.hpp"

namespace slope {

void PJConstraintSolver::solve_constraint(ConstraintData& c)
{
    float prev_lambda = c.lambda;
    float dot = c.lambda * c.cfm_inv_dt;

    if (c.body2_idx >= 0) {
        auto& b1 = m_bodies[c.body1_idx];
        auto& b2 = m_bodies[c.body2_idx];

        dot += c.jacobian1[0].dot(b1.inv_m_f[0]) + c.jacobian1[1].dot(b1.inv_m_f[1]);
        dot += c.jacobian2[0].dot(b2.inv_m_f[0]) + c.jacobian2[1].dot(b2.inv_m_f[1]);

    } else {
        auto& b1 = m_bodies[c.body1_idx];
        dot += c.jacobian1[0].dot(b1.inv_m_f[0]) + c.jacobian1[1].dot(b1.inv_m_f[1]);
    }

    float delta = (c.rhs - dot) * c.inv_diag;
    c.lambda = clamp(c.min_bound, prev_lambda + delta, c.max_bound);
    delta = c.lambda - prev_lambda;
    c.delta_lambda = delta;
}

void PJConstraintSolver::solve_iterations()
{
    for(uint32_t iter = 0; iter < m_config.iteration_count; ++iter) {

        for (auto& c : m_constraints[(int)ConstraintGroup::Friction]) {
            float normal_lambda = m_constraints[(int)ConstraintGroup::Normal][c.normal_constr_idx].lambda;
            float bound = c.friction_ratio * fabsf(normal_lambda);
            c.min_bound = -bound;
            c.max_bound = bound;
        }

        for (auto& c : m_constraints[(int)ConstraintGroup::Normal]) {
            solve_constraint(c);
        }

        for (auto& c : m_constraints[(int)ConstraintGroup::Friction]) {
            solve_constraint(c);
        }

        for (auto& b : m_bodies) {
            b.inv_m_f[0].set_zero();
            b.inv_m_f[1].set_zero();
        }

        for (auto& container : m_constraints) {
            for (auto& c: container) {
                auto& b1 = m_bodies[c.body1_idx];

                b1.inv_m_f[0] += c.inv_m_j1[0] * c.lambda;
                b1.inv_m_f[1] += c.inv_m_j1[1] * c.lambda;

                if (c.body2_idx >= 0) {
                    auto& b2 = m_bodies[c.body2_idx];
                    b2.inv_m_f[0] += c.inv_m_j2[0] * c.lambda;
                    b2.inv_m_f[1] += c.inv_m_j2[1] * c.lambda;
                }
            }
        }
    }
}

} // slope
