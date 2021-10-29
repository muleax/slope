#include "app/render/camera.hpp"
#include "app/scene/transform.hpp"

#include <app/../../deps/glfw/glfw/deps/linmath.h>

namespace slope::app {

REGISTER_COMPONENT(CameraComponent);

Camera::Camera() {
    rebuild();
}

void Camera::set_transform(const Mat44& value) {
    m_transform = value;
}

void Camera::set_view_direction(const Vec3& dir, const Vec3& up) {
    m_view_direction = dir;
    m_view_up = up;
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
    const Vec3& pos = m_transform.translation();
    Vec3 at = pos + m_view_direction;

    mat4x4 v;
    vec3 eye = {pos.x, pos.y, pos.z};
    vec3 center = {at.x, at.y, at.z};
    vec3 up = {m_view_up.x, m_view_up.y, m_view_up.z};
    mat4x4_look_at(v, eye, center, up);

    mat4x4 p;
    mat4x4_perspective(p, m_fov, m_aspect_ratio, m_near_plane, m_far_plane);

    auto* pvp = reinterpret_cast<mat4x4*>(&m_view_proj);
    mat4x4_mul(*pvp, p, v);
}

void CameraSystem::update(float dt) {
    for (auto e : view<CameraComponent, TransformComponent>()) {
        auto& matrix = w().get_component<TransformComponent>(e)->transform;

        auto& camera = w().get_component_for_write<CameraComponent>(e)->camera;
        camera.set_transform(matrix);
        camera.rebuild();
    }
}

} // slope::app
