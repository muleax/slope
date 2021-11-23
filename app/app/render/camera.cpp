#include "app/render/camera.hpp"
#include "app/scene/transform.hpp"
#include "slope/debug/log.hpp"

namespace slope::app {

REGISTER_COMPONENT(CameraComponent);
REGISTER_COMPONENT(CameraControllerComponent);

// helper
static Mat44 perspective_rh(float fov_y, float aspect, float z_near, float z_far) {
    SL_ASSERT(std::isfinite(fov_y));
    SL_ASSERT(std::isfinite(aspect));
    SL_ASSERT(std::isfinite(z_near));
    SL_ASSERT(std::isfinite(z_far));
    SL_ASSERT(!equal(aspect, 0.f));
    SL_ASSERT(!equal(fov_y, 0.f));
    SL_ASSERT(!equal(z_near, z_far));

    float tan_half_fov_y = std::tan(fov_y * 0.5f);
    return Mat44(1.f / (aspect * tan_half_fov_y), 0.f,                  0.f,                               0.f,
                 0.f,                             1.f / tan_half_fov_y, 0.f,                               0.f,
                 0.f,                             0.f,                  z_far / (z_near - z_far),          -1.f,
                 0.f,                             0.f,                  z_far * z_near / (z_near - z_far), 0.f);
}

Camera::Camera() {
    rebuild();
}

void Camera::set_transform(const Mat44& value) {
    m_transform = value;
}

void Camera::set_fov(float value) {
    m_fov = value;
}

void Camera::set_near_plane(float value) {
    m_near_plane = value;
}

void Camera::set_far_plane(float value) {
    m_far_plane = value;
}

void Camera::set_aspect_ratio(float value) {
    m_aspect_ratio = value;
}

void Camera::rebuild() {
    auto p = perspective_rh(m_fov, m_aspect_ratio, m_near_plane, m_far_plane);
    m_view_proj = m_transform.inverted() * p;
}

void CameraSystem::update(float dt) {
    for (auto e : view<CameraComponent, CameraControllerComponent, TransformComponent>()) {
        auto* tc = w().modify<TransformComponent>(e);
        auto* cam = w().modify<CameraComponent>(e);
        auto* ctl = w().modify<CameraControllerComponent>(e);

        Vec3 vel_dir;
        if (ctl->move_fwd && !ctl->move_bkwd) {
            vel_dir.z = -1.f;
        } else if (!ctl->move_fwd && ctl->move_bkwd) {
            vel_dir.z = 1.f;
        }

        if (ctl->move_left && !ctl->move_right) {
            vel_dir.x = -1.f;
        } else if (!ctl->move_left && ctl->move_right) {
            vel_dir.x = 1.f;
        }

        // TODO: optimize
        auto pitch_rot = Mat44::rotation({1.f, 0.f, 0.f}, ctl->pitch);
        auto yaw_rot = Mat44::rotation({0.f, 1.f, 0.f}, -ctl->yaw);
        auto rot = pitch_rot * yaw_rot;

        Vec3 new_pos = tc->transform.translation() + rot.apply_normal(vel_dir) * (ctl->velocity * dt);

        tc->transform = rot * Mat44::translate(new_pos);

        cam->camera.set_transform(tc->transform);
        cam->camera.rebuild();
    }
}

} // slope::app
