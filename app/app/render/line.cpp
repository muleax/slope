#include "app/render/line.hpp"
#include "glad/gl.h"

namespace slope::app {

void LineShader::cache_attribute_locations() {
    m_view_proj_loc = location(UniformLayout::VIEW_PROJECTION);
}

LineRenderer::LineRenderer() {
    glGenBuffers(1, &m_line_buffer);
    glGenVertexArrays(1, &m_line_vao);

    glBindVertexArray(m_line_vao);

    glBindBuffer(GL_ARRAY_BUFFER, m_line_buffer);

    auto stride = static_cast<GLsizei>(sizeof(LinePoint));

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(0));

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(sizeof(Vec3)));

    glBindVertexArray(0);
}

LineRenderer::~LineRenderer() {
    glDeleteBuffers(1, &m_line_buffer);
    glDeleteVertexArrays(1, &m_line_vao);
}

void LineRenderer::draw(const LinePoint* data, size_t count) const {

    glBindVertexArray(m_line_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_line_buffer);

    glBufferData(GL_ARRAY_BUFFER, count * sizeof(LinePoint), data, GL_STREAM_DRAW);

    glDrawArrays(GL_LINES, 0, count);

    glBindVertexArray(0);
}

} // slope::app
