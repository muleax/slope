#pragma once
#include "slope/dynamics/joint/base_joint.hpp"

namespace slope {

class SphericalJoint : public WarmStartJoint<4> {
public:
    using WarmStartJoint::WarmStartJoint;

    void        set_damping(const float value) { m_damping = value; }
    void        set_anchor1(const vec3& value) { m_anchor1 = value; }
    void        set_anchor2(const vec3& value) { m_anchor2 = value; }

    const vec3& anchor1() const { return m_anchor1; }
    const vec3& anchor2() const { return m_anchor2; }

    void        apply_constraints(ConstraintSolver* solver) final;

private:
    vec3 m_anchor1;
    vec3 m_anchor2;

    float m_damping = 0.f;
};

} // slope
