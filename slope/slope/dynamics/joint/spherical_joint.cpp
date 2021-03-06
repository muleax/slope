#include "slope/dynamics/joint/spherical_joint.hpp"

namespace slope {

void SphericalJoint::apply_constraints(ConstraintSolver* solver)
{
    auto& transform1 = m_body1->transform();

    ConstraintGeom geom;
    geom.p1 = transform1.apply_point(m_anchor1);
    geom.p2 = m_body2 ? m_body2->transform().apply_point(m_anchor2) : m_anchor2;

    for (int i = 0; i < 3; i++) {
        geom.axis = transform1.apply_to_unit_axis(i);
        auto conf = Constraint::stabilized_bilateral(m_body1, m_body2, geom);
        conf.erp = m_erp;
        add_constraint_warm(i, solver, conf);
    }

    if (m_damping > 0.f) {
        vec3 dw = m_body1->ang_velocity();
        if (m_body2)
            dw -= m_body2->ang_velocity();

        Constraint damping_conf;
        damping_conf.body1 = m_body1;
        damping_conf.body2 = m_body2;
        damping_conf.jacobian1[0] = dw;
        damping_conf.jacobian2[1] = -dw;
        damping_conf.min_bound = -m_damping;
        damping_conf.max_bound = m_damping;

        add_constraint_warm(3, solver, damping_conf);
    }
}

} // slope
