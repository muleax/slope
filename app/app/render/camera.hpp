#pragma once
#include "app/ecs/system.hpp"
#include "slope/math/matrix44.hpp"

namespace slope::app {

class Camera {
public:
    Camera();

    void set_transform(const mat44& value);
    void set_fov(float value);
    void set_near_plane(float value);
    void set_far_plane(float value);
    void set_aspect_ratio(float value);

    void rebuild();

    const mat44& view_proj() const { return m_view_proj; }

private:
    mat44 m_transform;
    float m_fov = 1.57f;
    float m_near_plane = 0.01f;
    float m_far_plane = 1000.f;
    float m_aspect_ratio = 1.f;

    mat44 m_view_proj;
};

struct CameraComponent : public Component<CameraComponent> {
    Camera camera;
};

struct CameraControllerComponent : public Component<CameraControllerComponent> {
    float velocity = 8.f;

    bool move_fwd = false;
    bool move_bkwd = false;
    bool move_left = false;
    bool move_right = false;

    float yaw_sensivity = 0.003f;
    float pitch_sensivity = 0.003f;

    float yaw = 0.f;
    float pitch = 0.f;

    void rotate(float yaw_delta, float pitch_delta) {
        yaw = fmodf(PI + yaw + yaw_delta * yaw_sensivity, 2.f * PI) - PI;
        pitch = fmodf(PI + pitch + pitch_delta * pitch_sensivity, 2.f * PI) - PI;
    }
};

class CameraSystem : public System {
public:
    using System::System;
    void update(float dt) override;
};

} // slope::app
