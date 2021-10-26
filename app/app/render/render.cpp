#include "app/render/render.hpp"
#include "slope/debug/assert.hpp"
#include "glad/gl.h"

namespace slope::app {

Render::Render() {
    glGenBuffers(1, &m_instancing_buffer);
}

Render::~Render() {
    glDeleteBuffers(1, &m_instancing_buffer);
}

void Render::draw_mesh(const Mesh& mesh, const Material& material, VectorView<Mat44> instancing_data) {
    draw_mesh(mesh, material, instancing_data.data(), instancing_data.size());
}

void Render::draw_mesh(const Mesh& mesh, const Material& material, const Mat44* instancing_data, size_t instance_count) {

    material.shader().use();

    mesh.bind(m_instancing_buffer);

    glBindBuffer(GL_ARRAY_BUFFER, m_instancing_buffer);
    //glBufferData(GL_ARRAY_BUFFER, 30000 * sizeof(Mat44), nullptr, GL_STREAM_DRAW);
    auto instance_buffer_size = static_cast<GLsizei>(instance_count * sizeof(Mat44));
    glBufferData(GL_ARRAY_BUFFER, instance_buffer_size, instancing_data, GL_STREAM_DRAW);
    //glBufferSubData(GL_ARRAY_BUFFER, 0, instance_buffer_size, instancing_data);

    glDrawElementsInstanced(GL_TRIANGLES, mesh.size(), GL_UNSIGNED_INT, nullptr, static_cast<GLsizei>(instance_count));

    mesh.unbind();
}

} // slope::app
