#pragma once
#include "app/render/config.hpp"
#include "app/render/shader.hpp"
#include "slope/math/vector2.hpp"
#include "slope/math/vector3.hpp"
#include "slope/math/vector4.hpp"
#include "slope/containers/vector.hpp"
#include <memory>

namespace slope::app {

class Mesh {
public:
    struct Vertex {
        vec3 position;
        vec3 normal;
        vec4 color;
        vec2 tex_coords;
    };

    Mesh(VectorView<Vertex> vertices, VectorView<uint32_t> indices);
    ~Mesh();

    int             size() const { return m_size; }

    void            bind(RenderHandle instancing_buffer) const;
    void            unbind() const;

private:
    RenderHandle m_vao;
    RenderHandle m_vbo;
    RenderHandle m_ebo;
    int          m_size = 0;
};

class MeshShader : public Shader
{
public:
    struct AttributeLayout {
        static constexpr uint32_t   POSITION = 0;
        static constexpr uint32_t   NORMAL = 1;
        static constexpr uint32_t   COLOR = 2;
        static constexpr uint32_t   TEX_COORDS = 3;
        static constexpr uint32_t   MODEL_0 = 4;
        static constexpr uint32_t   MODEL_1 = 5;
        static constexpr uint32_t   MODEL_2 = 6;
        static constexpr uint32_t   MODEL_3 = 7;
    };

    struct UniformLayout {
        static constexpr char VIEW_PROJECTION[] = "view_projection";
        static constexpr char LIGHT_POSITION[] = "light_position";
        static constexpr char AMBIENT_STRENGTH[] = "ambient_strength";
        static constexpr char COLOR[] = "uniform_color";
    };

    using Shader::Shader;

    void set_view_projection(const mat44& value) const { set(m_view_proj_loc, value); }
    void set_ligh_position(const vec3& value) const { set(m_light_pos_loc, value); }

    void set_ambient_strength(float value) const { set(m_ambient_strength_loc, value); }
    void set_color(const vec3& value) const { set(m_color_loc, value); }

private:
    void cache_attribute_locations() override;

    Location m_view_proj_loc = 0;
    Location m_light_pos_loc = 0;
    Location m_ambient_strength_loc = 0;
    Location m_color_loc = 0;
};

class MeshRenderer {
public:
    MeshRenderer();
    ~MeshRenderer();

    void draw(const Mesh& mesh, const mat44* instancing_data, size_t instance_count) const;

private:
    RenderHandle m_instancing_buffer = 0;
};

} // slope::app
