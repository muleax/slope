#include "app/render/render_system.hpp"
#include "app/ecs/world.hpp"
#include "slope/debug/log.hpp"
#include <algorithm>

#include <app/../../deps/glfw/glfw/deps/linmath.h>

namespace slope::app {

Mat44 generate_mvp(const Vec3& offset, float time) {
    Mat44 res;

    mat4x4 v;
    vec3 eye = {0.f, 1.f, -2.f};
    vec3 center = {0.f, 0.f, 0.f};
    vec3 up = {0.f, 1.f, 0.f };
    mat4x4_look_at(v, eye, center, up);

    mat4x4 p;
    float aspect = 640/(float)480;
    mat4x4_perspective(p, 1.57f, aspect, 1.f, 1000000.f);

    mat4x4 m;
    mat4x4_identity(m);
    mat4x4_translate(m, offset.x, offset.y, offset.z);
    mat4x4_rotate_Z(m, m, time + 1.4f);

    //Apply the transformations. mvp=p*v*m.

    auto* pmvp = reinterpret_cast<mat4x4*>(&res);
    mat4x4_mul(*pmvp, p, v);
    mat4x4_mul(*pmvp, *pmvp, m);

    return res;
}

REGISTER_COMPONENT(RenderSingleton);
REGISTER_COMPONENT(RenderComponent);

void RenderSystem::update(float dt) {
    m_time += dt;

    auto render_view = view<RenderComponent>();

    if (render_view.empty()) {
        return;
    }

    for (auto e : render_view) {
        auto* rc = w().get_component_for_write<RenderComponent>(e);
        m_queue.push_back(rc);
    }

    // TODO: Z-ordering
    std::sort(m_queue.begin(), m_queue.end(), [](RenderComponent* a, RenderComponent* b) {
        return a->mesh.get() > b->mesh.get() || a->material.get() > b->material.get(); });

    auto render_single = w().get_singleton_for_write<RenderSingleton>();
    auto& render = render_single->render;
    auto max_count = render_single->max_instancing_count;

    RenderComponent* prev = nullptr;
    for (RenderComponent* rc : m_queue) {
        if (m_mvp.size() >= max_count || (prev && (rc->mesh.get() != prev->mesh.get() || rc->material.get() != prev->material.get()))) {
            render.draw_mesh(*prev->mesh, *prev->material, m_mvp);
            m_mvp.clear();
        }

        m_mvp.push_back(generate_mvp(rc->offs, m_time));
        prev = rc;
    }

    render.draw_mesh(*prev->mesh, *prev->material, m_mvp);
    m_mvp.clear();
    m_queue.clear();
}

} // slope::app