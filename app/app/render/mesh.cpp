#include "mesh.hpp"
#include "slope/debug/log.hpp"
#include "glad/gl.h"
#include "glm/mat2x4.hpp"

namespace slope::app {

Mesh::Mesh(VectorView<Vertex> vertices, VectorView<uint32_t> indices, RenderHandle instancing_buffer)
        : m_instancing_buffer(instancing_buffer)
        , m_size(static_cast<int>(indices.size())){
    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);
    glGenBuffers(1, &m_ebo);

    glBindVertexArray(m_vao);

    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(uint32_t), indices.data(), GL_STATIC_DRAW);

    auto stride = static_cast<GLsizei>(sizeof(Vertex));

    glEnableVertexAttribArray(ShaderLayout::ATTR_POSITION);
    glVertexAttribPointer(ShaderLayout::ATTR_POSITION, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(0));

    glEnableVertexAttribArray(ShaderLayout::ATTR_NORMAL);
    glVertexAttribPointer(ShaderLayout::ATTR_NORMAL, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(sizeof(Vec3)));

    glEnableVertexAttribArray(ShaderLayout::ATTR_COLOR);
    glVertexAttribPointer(ShaderLayout::ATTR_COLOR, 4, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(2 * sizeof(Vec3)));

    glEnableVertexAttribArray(ShaderLayout::ATTR_TEX_COORDS);
    glVertexAttribPointer(ShaderLayout::ATTR_TEX_COORDS, 2, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(2 * sizeof(Vec3) + sizeof(Vec4)));

    glBindBuffer(GL_ARRAY_BUFFER, m_instancing_buffer);

    auto inst_stride = static_cast<GLsizei>(4 * sizeof(Vec4));

    glEnableVertexAttribArray(ShaderLayout::ATTR_MVP_0);
    glVertexAttribPointer(ShaderLayout::ATTR_MVP_0, 4, GL_FLOAT, GL_FALSE, inst_stride, reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(ShaderLayout::ATTR_MVP_1);
    glVertexAttribPointer(ShaderLayout::ATTR_MVP_1, 4, GL_FLOAT, GL_FALSE, inst_stride, reinterpret_cast<void*>(sizeof(Vec4)));
    glEnableVertexAttribArray(ShaderLayout::ATTR_MVP_2);
    glVertexAttribPointer(ShaderLayout::ATTR_MVP_2, 4, GL_FLOAT, GL_FALSE, inst_stride, reinterpret_cast<void*>(2 * sizeof(Vec4)));
    glEnableVertexAttribArray(ShaderLayout::ATTR_MVP_3);
    glVertexAttribPointer(ShaderLayout::ATTR_MVP_3, 4, GL_FLOAT, GL_FALSE, inst_stride, reinterpret_cast<void*>(3 * sizeof(Vec4)));

    glVertexAttribDivisor(ShaderLayout::ATTR_MVP_0, 1);
    glVertexAttribDivisor(ShaderLayout::ATTR_MVP_1, 1);
    glVertexAttribDivisor(ShaderLayout::ATTR_MVP_2, 1);
    glVertexAttribDivisor(ShaderLayout::ATTR_MVP_3, 1);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

Mesh::~Mesh() {
    glDeleteVertexArrays(1, &m_vao);
    glDeleteBuffers(1, &m_vbo);
    glDeleteBuffers(1, &m_ebo);
}

} // slope::app