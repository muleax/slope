#pragma once
#include "app/render/mesh.hpp"
#include "app/render/line.hpp"
#include <memory>

namespace slope::app {

struct DefaultShaders {
    static std::shared_ptr<MeshShader> mesh_shader();
    static std::shared_ptr<LineShader> line_shader();
};

} // slope::app
