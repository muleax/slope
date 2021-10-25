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

void Render::draw_mesh(const Mesh& mesh, const Material& material, VectorView<Mat44> instancing_data) const {
    draw_mesh(mesh, material, instancing_data.data(), instancing_data.size());
}

void Render::draw_mesh(const Mesh& mesh, const Material& material, const Mat44* instancing_data, size_t instance_count) const {
    SL_ASSERT(m_instancing_buffer == mesh.instancing_buffer());

    material.shader().use();

    auto instance_buffer_size = static_cast<GLsizei>(instance_count * sizeof(Mat44));

    glBindBuffer(GL_ARRAY_BUFFER, m_instancing_buffer);
    glBufferData(GL_ARRAY_BUFFER, instance_buffer_size, instancing_data, GL_STREAM_DRAW);

    glBindVertexArray(mesh.vertex_array());

    glDrawElementsInstanced(GL_TRIANGLES, mesh.size(), GL_UNSIGNED_INT, nullptr, static_cast<GLsizei>(instance_count));
    //glDrawElements(GL_TRIANGLES, index_count, GL_UNSIGNED_INT, 0);

    glBufferData(GL_ARRAY_BUFFER, instance_buffer_size, nullptr, GL_STREAM_DRAW);

    glBindVertexArray(0);
}

} // slope::app
