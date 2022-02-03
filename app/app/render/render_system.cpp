#include "app/render/render_system.hpp"
#include "app/render/camera.hpp"
#include "app/render/default_shaders.hpp"
#include "app/scene/transform.hpp"
#include "slope/debug/assert.hpp"
#include "glad/gl.h"
#include <algorithm>

namespace slope::app {

REGISTER_COMPONENT(RenderSingleton);
REGISTER_COMPONENT(RenderComponent);
REGISTER_COMPONENT(DebugDrawComponent);
REGISTER_COMPONENT(LightSourceComponent);

void RenderSystem::draw_mesh_instanced(
        RenderComponent* rc, const CameraComponent* cam, const RenderSingleton* rs, const vec3& light_pos) {

    auto& material = *rc->material;
    material.use();

    auto& shader = material.shader();
    shader.set_view_projection(cam->camera.view_proj());
    shader.set_ligh_position(light_pos);

    if (rs->wireframe)
        shader.set_ambient_strength(1.f);

    m_mesh_renderer.draw(*rc->mesh, m_model_matrices.data(), m_model_matrices.size());
}

void RenderSystem::scene_draw() {
    auto render_view = view<RenderComponent, TransformComponent>();
    auto light_view = view<LightSourceComponent, TransformComponent>();
    auto cam_view = view<CameraComponent>();

    if (render_view.empty() || light_view.empty() || cam_view.empty()) {
        return;
    }

    auto render_single = w().modify_singleton<RenderSingleton>();
    if (render_single->wireframe)
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    else
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    glEnable(GL_DEPTH_TEST);

    for (auto e : render_view) {
        auto* rc = w().modify<RenderComponent>(e);
        m_queue.push_back({e, rc});
    }

    // TODO: Z-ordering
    std::sort(m_queue.begin(), m_queue.end(), [](QueueEntry& a, QueueEntry& b) {
        return a.rc->mesh.get() > b.rc->mesh.get() || a.rc->material.get() > b.rc->material.get(); });

    auto max_count = render_single->max_instancing_count;

    auto& light_pos = w().get<TransformComponent>(*light_view.begin())->transform.translation();
    auto* cam = w().get<CameraComponent>(*cam_view.begin());

    RenderComponent* prev = nullptr;
    for (auto [e, rc] : m_queue) {
        if (m_model_matrices.size() >= max_count || (prev && (rc->mesh.get() != prev->mesh.get() || rc->material.get() != prev->material.get()))) {
            draw_mesh_instanced(prev, cam, render_single, light_pos);
            m_model_matrices.clear();
        }

        m_model_matrices.push_back(w().get<TransformComponent>(e)->transform);
        prev = rc;
    }

    draw_mesh_instanced(prev, cam, render_single, light_pos);
    m_model_matrices.clear();
    m_queue.clear();
}

void RenderSystem::debug_draw() {
    auto debug_view = view<DebugDrawComponent>();
    auto cam_view = view<CameraComponent>();

    if (debug_view.empty() || cam_view.empty()) {
        return;
    }

    if (!m_line_shader)
        m_line_shader = DefaultShaders::line_shader();

    auto& view_proj = w().get<CameraComponent>(*cam_view.begin())->camera.view_proj();

    m_line_shader->use();
    m_line_shader->set_view_projection(view_proj);

    for (auto e : debug_view) {
        auto* dc = w().modify<DebugDrawComponent>(e);
        m_line_renderer.draw(dc->drawer->lines_points().data(), dc->drawer->lines_points().size());
    }
}

void RenderSystem::update(float dt) {
    scene_draw();
    debug_draw();
}

} // slope::app
