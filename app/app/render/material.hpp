#pragma once
#include "app/render/shader.hpp"
#include "slope/math/matrix44.hpp"

namespace slope::app {

class Material {
public:
    Material(ShaderPtr shader) {
        m_shader = std::move(shader);
        m_ambient_strength_loc = m_shader->location(ShaderUniformLayout::AMBIENT_STRENGTH);
    }
    ~Material() = default;

    const Shader& shader() const { return *m_shader; }

    void use() const {
        m_shader->use();
        m_shader->set(m_ambient_strength_loc, m_ambient_strength);
    }

    void set_ambient_strength(float value) { m_ambient_strength = value; }

private:
    ShaderPtr m_shader;

    Shader::Location m_ambient_strength_loc = 0;
    float m_ambient_strength = 0.15f;
};

using MaterialPtr = std::shared_ptr<Material>;

} // slope::app