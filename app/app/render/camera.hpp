#pragma once
#include "app/ecs/system.hpp"
#include "slope/math/matrix44.hpp"

namespace slope::app {

class Camera {
public:
    Camera();

    void set_transform(const Mat44& value);
    void set_view_direction(const Vec3& dir, const Vec3& up);
    void set_fov(float value);
    void set_near_plane(float value);
    void set_far_plane(float value);
    void set_aspect_ratio(float value);

    void rebuild();

    const Mat44& view_proj() const { return m_view_proj; }

private:
    Mat44 m_transform;
    Vec3 m_view_direction = {0.f, 0.f, 1.f};
    Vec3 m_view_up = {0.f, 1.f, 0.f};
    float m_fov = 1.57f;
    float m_near_plane = 1.f;
    float m_far_plane = 1000.f;
    float m_aspect_ratio = 1.f;

    Mat44 m_view_proj;
};

class CameraComponent : public Component<CameraComponent> {
public:
    Camera camera;
};

class CameraSystem : public System {
public:
    using System::System;
    void update(float dt) override;
};

} // slope::app
