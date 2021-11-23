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
    }

    void set_ambient_strength(float value) { m_ambient_strength = value; }

private:
    std::shared_ptr<MeshShader> m_shader;

    float m_ambient_strength = 0.25f;
};

} // slope::app
