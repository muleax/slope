#include "slope/dynamics/joint.hpp"

namespace slope {

void SphericalJoint::apply_constraints(ConstraintSolver* solver)
{
    auto& transform1 = m_body1->transform();

    ConstraintGeom geom;
    geom.p1 = transform1.apply_point(m_anchor1);
    geom.p2 = m_body2 ? m_body2->transform().apply_point(m_anchor2) : m_anchor2;

    for (int i = 0; i < 3; i++) {
        auto& cache = m_cache[i];
        geom.axis = transform1.apply_to_unit_axis(i);
        auto conf = Constraint::stabilized_bilateral(m_body1, m_body2, geom);
        conf.init_lambda = cache.lambda * m_warmstarting_ratio;
        cache.constraint_id = solver->add_constraint(conf);
    }

    if (m_damping > 0.f) {
        auto& damping_cache = m_cache[3];

        Vec3 dw = m_body1->ang_velocity();
        if (m_body2)
            dw -= m_body2->ang_velocity();

        Constraint damping_conf;
        damping_conf.body1 = m_body1;
        damping_conf.body2 = m_body2;
        damping_conf.jacobian1[0] = dw;
        damping_conf.jacobian2[1] = -dw;
        damping_conf.min_bound = -m_damping;
        damping_conf.max_bound = m_damping;
        damping_conf.init_lambda = damping_cache.lambda * m_warmstarting_ratio;

        damping_cache.constraint_id = solver->add_constraint(damping_conf);
    }
}

} // slope
