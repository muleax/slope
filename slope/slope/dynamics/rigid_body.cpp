#include "slope/dynamics/rigid_body.hpp"
#include "slope/math/matrix33.hpp"

namespace slope {

void RigidBody::apply_gyroscopic_torque(float dt)
{
    // The idea is taken from Bullet Physics SDK. Please, see it for more details.
    // Euler explicit integration is unstable in many cases when computing gyroscopic forces.

    // Euler implicit integration:
    // f(x) := dx/dt
    // x' = x + h * f(x')

    // Newton iteration:
    // g(x) = 0
    // x' = x - inv(dg/dx[x]) * g(x)

    // Euler equations for rotational motion:
    // T = I * dw/dt + w x Iw

    // Euler implicit integration gives:
    // f(w) = inv(I) * (T - w x Iw)
    // w' = w + h * inv(I) * (T - w' x Iw')

    // Solve for new angular velocity using Newton iteration:
    // g(w) := Iw + h * w x Iw - (h * T + Iw0)
    // w' = w - inv(dg/dw[w]) * g(w)

    // Note:
    // d(u x v) / dt = du/dt x v + u x dv/dt

    // Jacobian dw/dw is Identity matrix
    // dg/dw = I + h * (cross[w] * I - cross[Iw])

    // Note that our explicit integrator ignores w x Iw term,
    // so we calculate correction in terms of angular velocity to address that (putting T = 0).
    // Just one iteration of Newton seems to be enough.

    // TODO: optimize
    auto& w = m_ang_velocity;

    Mat44 I = m_inv_transform * m_local_inertia * m_transform;

    auto Iw = I.apply_normal(w);

    Vec3 g = dt * w.cross(Iw);

    auto w_cross = Mat44{ Mat33::cross(w) };
    auto Iw_cross = Mat44{ Mat33::cross(Iw) };
    auto dgdw = I + dt * (I * w_cross - Iw_cross);

    auto dw = -dgdw.inverted().apply_normal(g);
    m_ang_velocity += dw;
}

void RigidBody::integrate(float dt)
{
    constexpr float ANG_VEL_EPSILON = 1e-8f;

    m_velocity = predict_velocity(dt);
    m_ang_velocity = predict_ang_velocity(dt);

    Vec3 new_pos = m_transform.translation() + m_velocity * dt;

    float ang_vel_magnitude = m_ang_velocity.length();

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
