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

    void draw_mesh(const Mesh& mesh, const Material& material, VectorView<Mat44> instancing_data) const;
    void draw_mesh(const Mesh& mesh, const Material& material, const Mat44* instancing_data, size_t instance_count) const;

    RenderHandle instancing_buffer() const { return m_instancing_buffer; }

private:
    RenderHandle m_instancing_buffer = 0;
};

} // slope::app