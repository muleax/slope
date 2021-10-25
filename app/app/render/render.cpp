#include "app/render/render.hpp"
#include "slope/debug/assert.hpp"
#include "glad/gl.h"

namespace slope::app {

Render::Render() {
    glGenBuffers(1, &m_instancing_buffer);

    glBindBuffer(GL_ARRAY_BUFFER, m_instancing_buffer);
    glBufferData(GL_ARRAY_BUFFER, m_max_instance_count * sizeof(Mat44), nullptr, GL_STREAM_DRAW);
}

Render::~Render() {
    glDeleteBuffers(1, &m_instancing_buffer);
}

void Render::draw_mesh(const Mesh& mesh, const Material& material, const Mat44& mvp) {
    material.shader().use();
    static auto loc = material.shader().location("uMVP");
    material.shader().set(loc, mvp);

    glBindVertexArray(mesh.vertex_array());

    glDrawElements(GL_TRIANGLES, mesh.size(), GL_UNSIGNED_INT, nullptr);

    glBindVertexArray(0);
}

void Render::draw_mesh(const Mesh& mesh, const Material& material, VectorView<Mat44> instancing_data) {
    draw_mesh(mesh, material, instancing_data.data(), instancing_data.size());
}

void Render::draw_mesh(const Mesh& mesh, const Material& material, const Mat44* instancing_data, size_t instance_count) {
    SL_ASSERT(m_instancing_buffer == mesh.instancing_buffer());

    material.shader().use();

    auto instance_buffer_size = static_cast<GLsizei>(instance_count * sizeof(Mat44));

    glBindBuffer(GL_ARRAY_BUFFER, m_instancing_buffer);
    //glBindBuffer(GL_ARRAY_BUFFER, particles_color_buffer);
    glBufferData(GL_ARRAY_BUFFER, m_max_instance_count * sizeof(Mat44), nullptr, GL_STREAM_DRAW);
    // Buffer orphaning, a common way to improve streaming perf. See above link for details.

    //if (m_offset + instance_buffer_size > 30000* sizeof(Mat44))
    //    m_offset = 0;

    //glBufferData(GL_ARRAY_BUFFER, instance_buffer_size, instancing_data, GL_STREAM_DRAW);
    glBufferSubData(GL_ARRAY_BUFFER, 0, instance_buffer_size, instancing_data);

    //m_offset += instance_buffer_size;

    glBindVertexArray(mesh.vertex_array());

    glDrawElementsInstanced(GL_TRIANGLES, mesh.size(), GL_UNSIGNED_INT, nullptr, static_cast<GLsizei>(instance_count));
    //glDrawElements(GL_TRIANGLES, index_count, GL_UNSIGNED_INT, 0);

    //glBindBuffer(GL_ARRAY_BUFFER, m_instancing_buffer);
    //glUnmapBuffer(GL_ARRAY_BUFFER);

    //glBufferData(GL_ARRAY_BUFFER, instance_buffer_size, nullptr, GL_STREAM_DRAW);
   //glMapBufferRange()

    glBindVertexArray(0);
    //glFinish();
}

} // slope::app
