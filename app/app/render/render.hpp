#pragma once
#include "app/render/config.hpp"
#include "app/render/mesh.hpp"
#include "app/render/material.hpp"
#include "slope/math/matrix44.hpp"
#include "slope/containers/vector.hpp"

namespace slope::app {

class Render {
public:
    Render();
    ~Render();

    void draw_mesh(const Mesh& mesh, const Material& material, const Mat44& mvp);

    void draw_mesh(const Mesh& mesh, const Material& material, VectorView<Mat44> instancing_data);
    void draw_mesh(const Mesh& mesh, const Material& material, const Mat44* instancing_data, size_t instance_count);

    RenderHandle instancing_buffer() const { return m_instancing_buffer; }
    size_t max_instance_count() const { return m_max_instance_count; }

private:
    RenderHandle m_instancing_buffer = 0;
    size_t m_max_instance_count = 10;
    size_t m_offset = 0;
};

} // slope::app