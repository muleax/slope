#pragma once
#include "app/ecs/system.hpp"
#include "app/ecs/component.hpp"
#include "app/render/render.hpp"
#include "app/render/mesh.hpp"
#include "app/render/material.hpp"

namespace slope::app {

struct RenderSingleton : public Component<RenderSingleton> {
    Render render;
    size_t max_instancing_count = 1024;
};

struct RenderComponent : public Component<RenderComponent> {
    MeshPtr mesh;
    MaterialPtr material;
    Vec3 offs;
};

class RenderSystem : public System {
public:
    using System::System;
    void update(float dt) override;

private:
    float m_time = 0.f;
    Vector<RenderComponent*> m_queue;
    Vector<Mat44> m_mvp;
};

} // slope::app