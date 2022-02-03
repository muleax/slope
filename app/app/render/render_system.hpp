#pragma once
#include "app/render/config.hpp"
#include "app/render/mesh.hpp"
#include "app/render/line.hpp"
#include "app/render/material.hpp"
#include "app/render/debug_drawer_impl.hpp"
#include "app/ecs/system.hpp"
#include "slope/math/matrix44.hpp"
#include "slope/containers/vector.hpp"

namespace slope::app {

class TransformComponent;
class CameraComponent;

struct RenderSingleton : public Component<RenderSingleton> {
    size_t max_instancing_count = 1024;
    bool wireframe = false;
};

struct RenderComponent : public Component<RenderComponent> {
    std::shared_ptr<Mesh> mesh;
    std::shared_ptr<Material> material;
};

struct DebugDrawComponent : public Component<DebugDrawComponent> {
    std::shared_ptr<DebugDrawerImpl> drawer;
};

struct LightSourceComponent : public Component<LightSourceComponent> {
};

class RenderSystem : public System {
public:
    using System::System;
    void update(float dt) override;

private:
    struct QueueEntry {
        Entity e;
        RenderComponent* rc;
    };

    void scene_draw();
    void debug_draw();
    void draw_mesh_instanced(
            RenderComponent* rc, const CameraComponent* cam, const RenderSingleton* rs, const vec3& light_pos);

    MeshRenderer m_mesh_renderer;
    Vector<QueueEntry> m_queue;
    Vector<mat44> m_model_matrices;

    LineRenderer m_line_renderer;
    std::shared_ptr<LineShader> m_line_shader;
};

} // slope::app
