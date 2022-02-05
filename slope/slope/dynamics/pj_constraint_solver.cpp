#include "slope/dynamics/pj_constraint_solver.hpp"

namespace slope {
/*
void PJConstraintSolver::solve_constraint_pj(ConstraintData& c, float& lambda)
{
    float prev_lambda = lambda;
    float dot = lambda * c.cfm_inv_dt;

    if (c.body2_idx >= 0) {
        auto& b1 = m_bodies[c.body1_idx];
        auto& b2 = m_bodies[c.body2_idx];

        dot += c.jacobian11.dot(b1.inv_m_f1) + c.jacobian12.dot(b1.inv_m_f2);
        dot += c.jacobian21.dot(b2.inv_m_f1) + c.jacobian22.dot(b2.inv_m_f2);

    } else {
        auto& b1 = m_bodies[c.body1_idx];
        dot += c.jacobian11.dot(b1.inv_m_f1) + c.jacobian12.dot(b1.inv_m_f2);
    }

    float delta = (c.rhs - dot) * c.inv_diag;
    lambda = clamp(c.min_bound, prev_lambda + delta, c.max_bound);
}

void PJConstraintSolver::solve_iterations_pj()
{
    for(uint32_t iter = 0; iter < m_config.iteration_count; ++iter) {

        auto& general_group = m_groups[(int)ConstraintGroup::General];

        for (auto friction_type : { ConstraintGroup::Friction1D, ConstraintGroup::Friction2D, ConstraintGroup::FrictionCone }) {
            auto& friction_group = m_groups[(int)friction_type];
            for (auto& c : friction_group->constraints) {
                float normal_lambda = general_group->lambda[c.normal_constr_idx];
                float bound = c.friction_ratio * fabsf(normal_lambda);
                c.min_bound = -bound;
                c.max_bound = bound;
            }
        }

        // Cone friction is not supported
        for (auto& container : m_groups) {
            auto lambda_it = container->lambda.begin();
            for (auto& c : container->constraints) {
                solve_constraint_pj(c, *lambda_it);
                ++lambda_it;
            }
        }

        for (auto& b : m_bodies) {
            b.inv_m_f1.set_zero();
            b.inv_m_f2.set_zero();
        }

        for (auto& container : m_groups) {
            auto lambda_it = container->lambda.begin();

            for (auto& c : container->constraints) {
                auto lambda =  *lambda_it;

                auto& b1 = m_bodies[c.body1_idx];

                b1.inv_m_f1 += c.inv_m_j11 * lambda;
                b1.inv_m_f2 += c.inv_m_j12 * lambda;

                if (c.body2_idx >= 0) {
                    auto& b2 = m_bodies[c.body2_idx];
                    b2.inv_m_f1 += c.inv_m_j21 * lambda;
                    b2.inv_m_f2 += c.inv_m_j22 * lambda;
                }

                ++lambda_it;
            }
        }
    }
}

void PJConstraintSolver::solve()
{
    prepare_data();
    solve_iterations_pj();
    apply_impulses();
}
*/
} // slope
