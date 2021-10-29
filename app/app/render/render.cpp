#include "app/render/render.hpp"
#include "app/render/camera.hpp"
#include "app/scene/transform.hpp"
#include "slope/debug/assert.hpp"
#include "glad/gl.h"
#include <algorithm>

namespace slope::app {

REGISTER_COMPONENT(RenderSingleton);
REGISTER_COMPONENT(RenderComponent);
REGISTER_COMPONENT(LightSourceComponent);

Renderer::Renderer() {
    glGenBuffers(1, &m_instancing_buffer);
}

Renderer::~Renderer() {
    glDeleteBuffers(1, &m_instancing_buffer);
}

void Renderer::draw_mesh(const Mesh& mesh, VectorView<Mat44> instancing_data) const {
    draw_mesh(mesh, instancing_data.data(), instancing_data.size());
}

void Renderer::draw_mesh(const Mesh& mesh, const Mat44* instancing_data, size_t instance_count) const {

    mesh.bind(m_instancing_buffer);

    glBindBuffer(GL_ARRAY_BUFFER, m_instancing_buffer);
    //glBufferData(GL_ARRAY_BUFFER, 30000 * sizeof(Mat44), nullptr, GL_STREAM_DRAW);
    auto instance_buffer_size = static_cast<GLsizei>(instance_count * sizeof(Mat44));
    glBufferData(GL_ARRAY_BUFFER, instance_buffer_size, instancing_data, GL_STREAM_DRAW);
    //glBufferSubData(GL_ARRAY_BUFFER, 0, instance_buffer_size, instancing_data);

    glDrawElementsInstanced(GL_TRIANGLES, mesh.size(), GL_UNSIGNED_INT, nullptr, static_cast<GLsizei>(instance_count));

    mesh.unbind();
}
/*
Mat44 generate_model(const Vec3& offset, float time) {
    Mat44 res;
    auto* pm = reinterpret_cast<mat4x4*>(&res);

    mat4x4_identity(*pm);
    mat4x4_translate(*pm, offset.x, offset.y, offset.z);
    mat4x4_rotate_Z(*pm, *pm, time + 1.4f);
    mat4x4_rotate_X(*pm, *pm, time + 1.4f);

    return res;
}
*/
void RenderSystem::update(float dt) {
    m_time += dt;

    auto render_view = view<RenderComponent, TransformComponent>();
    auto light_view = view<LightSourceComponent, TransformComponent>();
    auto cam_view = view<CameraComponent>();

    if (render_view.empty() || light_view.empty() || cam_view.empty()) {
        return;
    }

    for (auto e : render_view) {
        auto* rc = w().get_component_for_write<RenderComponent>(e);
        m_queue.push_back({e, rc});
    }

    // TODO: Z-ordering
    std::sort(m_queue.begin(), m_queue.end(), [](QueueEntry& a, QueueEntry& b) {
        return a.rc->mesh.get() > b.rc->mesh.get() || a.rc->material.get() > b.rc->material.get(); });

    auto render_single = w().get_singleton_for_write<RenderSingleton>();
    auto max_count = render_single->max_instancing_count;

    auto& light_pos = w().get_component<TransformComponent>(*light_view.begin())->transform.translation();
    auto& view_proj = w().get_component<CameraComponent>(*cam_view.begin())->camera.view_proj();

    auto draw_mesh = [this, &view_proj, &light_pos] (RenderComponent* rc) {
        auto& material = *rc->material;
        material.use();

        auto& shader = material.shader();
        shader.set_view_projection(view_proj);
        shader.set_ligh_position(light_pos);

        m_renderer.draw_mesh(*rc->mesh, m_model);
    };

    glEnable(GL_DEPTH_TEST);

    RenderComponent* prev = nullptr;
    for (auto [e, rc] : m_queue) {
        if (m_model.size() >= max_count || (prev && (rc->mesh.get() != prev->mesh.get() || rc->material.get() != prev->material.get()))) {
            draw_mesh(prev);
            m_model.clear();
        }

        m_model.push_back(w().get_component<TransformComponent>(e)->transform);
        prev = rc;
    }

    draw_mesh(prev);
    m_model.clear();
    m_queue.clear();
}

} // slope::app
