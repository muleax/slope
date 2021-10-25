#pragma once
#include "app/render/config.hpp"
#include "slope/math/vector2.hpp"
#include "slope/math/vector3.hpp"
#include "slope/math/vector4.hpp"
#include "slope/containers/vector.hpp"
#include <memory>


#include "glad/gl.h"

namespace slope::app {

class Mesh {
public:
    struct Vertex {
        Vec3 position;
        Vec3 normal;
        Vec4 color;
        Vec2 tex_coords;
    };

    Mesh(VectorView<Vertex> vertices, VectorView<uint32_t> indices, RenderHandle instancing_buffer);
    ~Mesh();

    RenderHandle    vertex_array() const { return m_vao; }
    RenderHandle    instancing_buffer() const { return m_instancing_buffer; }
    int             size() const { return m_size; }

    void draw () {

        glBindVertexArray(m_vao);

        glDrawElements(GL_TRIANGLES, m_size, GL_UNSIGNED_INT, 0);

        glBindVertexArray(0);
    }

private:
    RenderHandle m_vao;
    RenderHandle m_vbo;
    RenderHandle m_ebo;
    RenderHandle m_instancing_buffer;
    int          m_size = 0;
};

using MeshPtr = std::shared_ptr<Mesh>;

} // slope::app