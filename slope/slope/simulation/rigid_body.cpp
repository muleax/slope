#include "slope/simulation/rigid_body.hpp"

namespace slope {

void RigidBody::integrate(float dt) {
    constexpr float ANG_VEL_EPSILON = 1e-6f;

    m_velocity = predict_velocity(dt);
    m_ang_velocity = predict_ang_velocity(dt);

    Vec3 new_pos = m_transform.translation() + m_velocity * dt;

    float ang_vel_magnitude = m_velocity.length();

    if (ang_vel_magnitude > ANG_VEL_EPSILON) {
        m_transform.set_translation(Vec3::zero());

        Vec3 rot_axis = m_ang_velocity / ang_vel_magnitude;
        float angle = ang_vel_magnitude * dt;
        m_transform *= Mat44::rotation(rot_axis, angle);

        // TODO: optimize normalization
        m_transform.as_vec3(2) = m_transform.as_vec3(2).normalized();
        m_transform.as_vec3(0) = m_transform.as_vec3(1).cross(m_transform.as_vec3(2));
        m_transform.as_vec3(0) = m_transform.as_vec3(0).normalized();
        m_transform.as_vec3(1) = m_transform.as_vec3(2).cross(m_transform.as_vec3(0));

    } else {
        m_ang_velocity.set_zero();
    }

    m_transform.set_translation(new_pos);

    update_matrices();

    m_force.set_zero();
    m_torque.set_zero();
}

} // slope
