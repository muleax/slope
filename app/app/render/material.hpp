#pragma once
#include "app/resource/resource_cache.hpp"
#include "app/render/shader_program.hpp"

namespace slope::app {

class Material {
public:
    Material(ShaderProgramPtr shader) {
        m_shader = std::move(shader);
    }
    ~Material() = default;

    const ShaderProgram& shader() const { return *m_shader; }

private:
    ShaderProgramPtr m_shader;
};

using MaterialPtr = std::shared_ptr<Material>;

} // slope::app