#include "app/render/mesh.hpp"
#include "slope/debug/log.hpp"
#include "glad/gl.h"
#include "glm/mat2x4.hpp"

namespace slope::app {

void MeshShader::cache_attribute_locations() {
    m_view_proj_loc = location(UniformLayout::VIEW_PROJECTION);
    m_light_pos_loc = location(UniformLayout::LIGHT_POSITION);
    m_ambient_strength_loc = location(UniformLayout::AMBIENT_STRENGTH);
    m_color_loc = location(UniformLayout::COLOR);
}

Mesh::Mesh(VectorView<Vertex> vertices, VectorView<uint32_t> indices)
        : m_size(static_cast<int>(indices.size())){
    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);
    glGenBuffers(1, &m_ebo);

    glBindVertexArray(m_vao);

    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(uint32_t), indices.data(), GL_STATIC_DRAW);

    auto stride = static_cast<GLsizei>(sizeof(Vertex));

    glEnableVertexAttribArray(MeshShader::AttributeLayout::POSITION);
    glVertexAttribPointer(MeshShader::AttributeLayout::POSITION, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(0));

    glEnableVertexAttribArray(MeshShader::AttributeLayout::NORMAL);
    glVertexAttribPointer(MeshShader::AttributeLayout::NORMAL, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(sizeof(Vec3)));

    glEnableVertexAttribArray(MeshShader::AttributeLayout::COLOR);
    glVertexAttribPointer(MeshShader::AttributeLayout::COLOR, 4, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(2 * sizeof(Vec3)));

    glEnableVertexAttribArray(MeshShader::AttributeLayout::TEX_COORDS);
    glVertexAttribPointer(MeshShader::AttributeLayout::TEX_COORDS, 2, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(2 * sizeof(Vec3) + sizeof(Vec4)));

    glEnableVertexAttribArray(MeshShader::AttributeLayout::MODEL_0);
    glEnableVertexAttribArray(MeshShader::AttributeLayout::MODEL_1);
    glEnableVertexAttribArray(MeshShader::AttributeLayout::MODEL_2);
    glEnableVertexAttribArray(MeshShader::AttributeLayout::MODEL_3);

    glVertexAttribDivisor(MeshShader::AttributeLayout::MODEL_0, 1);
    glVertexAttribDivisor(MeshShader::AttributeLayout::MODEL_1, 1);
    glVertexAttribDivisor(MeshShader::AttributeLayout::MODEL_2, 1);
    glVertexAttribDivisor(MeshShader::AttributeLayout::MODEL_3, 1);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void Mesh::bind(RenderHandle instancing_buffer) const {
    glBindVertexArray(m_vao);

    auto stride = static_cast<GLsizei>(4 * sizeof(Vec4));

    glBindBuffer(GL_ARRAY_BUFFER, instancing_buffer);
    glVertexAttribPointer(MeshShader::AttributeLayout::MODEL_0, 4, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(0));
    glVertexAttribPointer(MeshShader::AttributeLayout::MODEL_1, 4, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(sizeof(Vec4)));
    glVertexAttribPointer(MeshShader::AttributeLayout::MODEL_2, 4, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(2 * sizeof(Vec4)));
    glVertexAttribPointer(MeshShader::AttributeLayout::MODEL_3, 4, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(3 * sizeof(Vec4)));
}

void Mesh::unbind() const {
    glBindVertexArray(0);
}

Mesh::~Mesh() {
    glDeleteVertexArrays(1, &m_vao);
    glDeleteBuffers(1, &m_vbo);
    glDeleteBuffers(1, &m_ebo);
}

MeshRenderer::MeshRenderer() {
    glGenBuffers(1, &m_instancing_buffer);
}

MeshRenderer::~MeshRenderer() {
    glDeleteBuffers(1, &m_instancing_buffer);
}

void MeshRenderer::draw(const Mesh& mesh, const Mat44* instancing_data, size_t instance_count) const {

    mesh.bind(m_instancing_buffer);

    glBindBuffer(GL_ARRAY_BUFFER, m_instancing_buffer);
    auto instance_buffer_size = static_cast<GLsizei>(instance_count * sizeof(Mat44));
    glBufferData(GL_ARRAY_BUFFER, instance_buffer_size, instancing_data, GL_STREAM_DRAW);

    glDrawElementsInstanced(GL_TRIANGLES, mesh.size(), GL_UNSIGNED_INT, nullptr, static_cast<GLsizei>(instance_count));

    mesh.unbind();
}


} // slope::app