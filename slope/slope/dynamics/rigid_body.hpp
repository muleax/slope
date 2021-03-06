#pragma once
#include "slope/math/math.hpp"

namespace slope {

class RigidBody {
public:
    RigidBody() = default;
    ~RigidBody() = default;

    void            set_mass(float mass);
    float           mass() const { return m_mass; }
    float           inv_mass() const { return m_inv_mass; }

    void            set_local_inertia(const vec3& diagonal);
    void            set_local_inertia(const mat44& local_inertia);
    const mat44&    local_inertia() const { return m_local_inertia; }
    const mat44&    inv_local_inertia() const { return m_inv_local_inertia; }
    const mat44&    inv_inertia() const { return m_inv_inertia; }

    void            set_transform(const mat44& transform);
    const mat44&    transform() const { return m_transform; }
    const mat44&    inv_transform() const { return m_inv_transform; }

    void            set_velocity(const vec3& velocity);
    const vec3&     velocity() const { return m_velocity; }

    void            set_ang_velocity(const vec3& ang_velocity);
    const vec3&     ang_velocity() const { return m_ang_velocity; }

    vec3            predict_velocity(float dt) const;
    vec3            predict_ang_velocity(float dt) const;
    vec3            point_velocity(const vec3& point) const;

    void            apply_impulse(const vec3& point, const vec3& impulse);
    void            apply_impulse_to_com(const vec3& impulse);

    void            apply_force(const vec3& point, const vec3& force);
    void            apply_force_to_com(const vec3& force);
    const vec3&     force() const { return m_force; }

    void            apply_torque(const vec3& torque);
    const vec3&     torque() const { return m_torque; }

    void            apply_gyroscopic_torque(float dt);

    void            integrate(float dt);

    int             in_solver_index() const { return m_in_solver_index; }
    void            set_in_solver_index(int index) { m_in_solver_index = index; }

private:
    void update_matrices();

    float m_mass = 1.f;
    float m_inv_mass = 1.f;
    mat44 m_local_inertia = mat44::identity();
    mat44 m_inv_local_inertia = mat44::identity();
    mat44 m_inv_inertia = mat44::identity();

    mat44 m_transform = mat44::identity();
    mat44 m_inv_transform = mat44::identity();

    vec3 m_velocity;
    vec3 m_ang_velocity;

    vec3 m_force;
    vec3 m_torque;

    int m_in_solver_index = -1;
};

inline void RigidBody::set_mass(float mass) {
    m_mass = mass;
    m_inv_mass = 1.f / mass;
}

inline void RigidBody::set_local_inertia(const vec3& diagonal) {
    set_local_inertia(mat44::scale(diagonal));
}

inline void RigidBody::set_local_inertia(const mat44& local_inertia) {
    m_local_inertia = local_inertia;
    //m_inv_local_inertia = m_local_inertia.inverted_orthonormal();
    m_inv_local_inertia = m_local_inertia.inverted();
    m_inv_inertia = m_inv_transform * m_inv_local_inertia * m_transform;
}

inline void RigidBody::set_transform(const mat44& transform) {
    m_transform = transform;
    update_matrices();
}

inline void RigidBody::set_velocity(const vec3& velocity) {
    m_velocity = velocity;
}

inline void RigidBody::set_ang_velocity(const vec3& ang_velocity) {
    m_ang_velocity = ang_velocity;
}

inline vec3 RigidBody::predict_velocity(float dt) const {
    return m_velocity + m_force * (dt * m_inv_mass);
}

inline vec3 RigidBody::predict_ang_velocity(float dt) const {
    return m_ang_velocity + m_inv_inertia.apply_normal(m_torque) * dt;
}

inline vec3 RigidBody::point_velocity(const vec3& point) const {
    auto r = point - m_transform.translation();
    return m_velocity + m_ang_velocity.cross(r);
}

inline void RigidBody::apply_impulse(const vec3& point, const vec3& impulse) {
    m_velocity += impulse * m_inv_mass;
    auto r = point - m_transform.translation();
    m_ang_velocity += m_inv_inertia.apply_normal(r.cross(impulse));
}

inline void RigidBody::apply_impulse_to_com(const vec3& impulse) {
    m_velocity += impulse * m_inv_mass;
}

inline void RigidBody::apply_force(const vec3& point, const vec3& force) {
    m_force += force;
    auto r = point - m_transform.translation();
    m_torque += r.cross(force);
}

inline void RigidBody::apply_force_to_com(const vec3& force) {
    m_force += force;
}

inline void RigidBody::apply_torque(const vec3& torque) {
    m_torque += torque;
}

inline void RigidBody::update_matrices() {
    m_inv_transform = m_transform.inverted_orthonormal();
    m_inv_inertia = m_inv_transform * m_inv_local_inertia * m_transform;
}

} // slope
