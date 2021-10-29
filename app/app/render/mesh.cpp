#include "mesh.hpp"
#include "slope/debug/log.hpp"
#include "glad/gl.h"
#include "glm/mat2x4.hpp"

namespace slope::app {

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

    glEnableVertexAttribArray(ShaderAttributeLayout::POSITION);
    glVertexAttribPointer(ShaderAttributeLayout::POSITION, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(0));

    glEnableVertexAttribArray(ShaderAttributeLayout::NORMAL);
    glVertexAttribPointer(ShaderAttributeLayout::NORMAL, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(sizeof(Vec3)));

    glEnableVertexAttribArray(ShaderAttributeLayout::COLOR);
    glVertexAttribPointer(ShaderAttributeLayout::COLOR, 4, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(2 * sizeof(Vec3)));

    glEnableVertexAttribArray(ShaderAttributeLayout::TEX_COORDS);
    glVertexAttribPointer(ShaderAttributeLayout::TEX_COORDS, 2, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(2 * sizeof(Vec3) + sizeof(Vec4)));

    glEnableVertexAttribArray(ShaderAttributeLayout::MODEL_0);
    glEnableVertexAttribArray(ShaderAttributeLayout::MODEL_1);
    glEnableVertexAttribArray(ShaderAttributeLayout::MODEL_2);
    glEnableVertexAttribArray(ShaderAttributeLayout::MODEL_3);

    glVertexAttribDivisor(ShaderAttributeLayout::MODEL_0, 1);
    glVertexAttribDivisor(ShaderAttributeLayout::MODEL_1, 1);
    glVertexAttribDivisor(ShaderAttributeLayout::MODEL_2, 1);
    glVertexAttribDivisor(ShaderAttributeLayout::MODEL_3, 1);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void Mesh::bind(RenderHandle instancing_buffer) const {
    glBindVertexArray(m_vao);

    auto stride = static_cast<GLsizei>(4 * sizeof(Vec4));

    glBindBuffer(GL_ARRAY_BUFFER, instancing_buffer);
    glVertexAttribPointer(ShaderAttributeLayout::MODEL_0, 4, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(0));
    glVertexAttribPointer(ShaderAttributeLayout::MODEL_1, 4, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(sizeof(Vec4)));
    glVertexAttribPointer(ShaderAttributeLayout::MODEL_2, 4, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(2 * sizeof(Vec4)));
    glVertexAttribPointer(ShaderAttributeLayout::MODEL_3, 4, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(3 * sizeof(Vec4)));
}

void Mesh::unbind() const {
    glBindVertexArray(0);
}

Mesh::~Mesh() {
    glDeleteVertexArrays(1, &m_vao);
    glDeleteBuffers(1, &m_vbo);
    glDeleteBuffers(1, &m_ebo);
}

} // slope::app