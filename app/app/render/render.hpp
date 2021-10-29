#pragma once
#include "app/render/config.hpp"
#include "app/render/mesh.hpp"
#include "app/render/material.hpp"
#include "app/ecs/system.hpp"
#include "slope/math/matrix44.hpp"
#include "slope/containers/vector.hpp"

namespace slope::app {

class Renderer {
public:
    Renderer();
    ~Renderer();

    void draw_mesh(const Mesh& mesh, VectorView<Mat44> instancing_data) const;
    void draw_mesh(const Mesh& mesh, const Mat44* instancing_data, size_t instance_count) const;

private:
    RenderHandle m_instancing_buffer = 0;
};

struct RenderSingleton : public Component<RenderSingleton> {
    size_t max_instancing_count = 1024;
};

struct RenderComponent : public Component<RenderComponent> {
    MeshPtr mesh;
    MaterialPtr material;
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

    Renderer m_renderer;
    float m_time = 0.f;
    Vector<QueueEntry> m_queue;
    Vector<Mat44> m_model;
};

} // slope::app