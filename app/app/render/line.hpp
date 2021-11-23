#pragma once
#include "app/render/config.hpp"
#include "app/render/shader.hpp"

namespace slope::app {

struct LinePoint {
    Vec3 position;
    Vec3 color;
};

class LineShader : public Shader
{
public:
    struct AttributeLayout {
        static constexpr uint32_t   POSITION = 0;
        static constexpr uint32_t   COLOR = 1;
    };

    struct UniformLayout {
        static constexpr char VIEW_PROJECTION[] = "view_projection";
    };

    using Shader::Shader;
    void set_view_projection(const Mat44& value) const { set(m_view_proj_loc, value); }

private:
    void cache_attribute_locations() override;

    Location m_view_proj_loc = 0;
};

class LineRenderer {
public:
    LineRenderer();
    ~LineRenderer();

    void draw(const LinePoint* data, size_t count) const;

private:
    RenderHandle m_line_buffer = 0;
    RenderHandle m_line_vao = 0;
};

} // slope::app
