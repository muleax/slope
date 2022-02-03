#pragma once
#include "app/render/mesh.hpp"
#include "slope/math/matrix44.hpp"

namespace slope::app {

class Material {
public:
    Material(std::shared_ptr<MeshShader> shader) {
        m_shader = std::move(shader);
    }
    ~Material() = default;

    const MeshShader& shader() const { return *m_shader; }

    void use() const {
        m_shader->use();
        m_shader->set_ambient_strength(m_ambient_strength);
        m_shader->set_color(m_color);
    }

    void set_ambient_strength(float value) { m_ambient_strength = value; }
    void set_color(const vec3& value) { m_color = value; }

private:
    std::shared_ptr<MeshShader> m_shader;

    float m_ambient_strength = 0.25f;
    vec3 m_color = {1.f, 1.f, 1.f};
};

} // slope::app
