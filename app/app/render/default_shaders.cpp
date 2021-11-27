#include "app/render/default_shaders.hpp"

namespace slope::app {

static const char* s_line_vertex_shader_text =
        "#version 410\n"
        "layout (location = 0) in vec3 position;\n"
        "layout (location = 1) in vec3 color;\n"
        "uniform mat4 view_projection;\n"
        "out vec3 f_color;\n"
        "void main()\n"
        "{\n"
        "    f_color = color;"
        "    gl_Position = view_projection * vec4(position, 1.0);\n"
        "}\n";

static const char* s_line_fragment_shader_text =
        "#version 410\n"
        "in vec3 f_color;\n"
        "out vec4 out_color;\n"
        "void main()\n"
        "{\n"
        "   out_color = vec4(f_color, 1.0);\n"
        "}\n";

static const char* s_vertex_shader_text =
        "#version 410\n"
        "layout (location = 0) in vec3 position;\n"
        "layout (location = 1) in vec3 normal;\n"
        "layout (location = 2) in vec4 color;\n"
        "layout (location = 3) in vec2 tex_coords;\n"
        "layout (location = 4) in mat4 model;\n"
        "uniform mat4 view_projection;\n"
        "out vec3 f_position;\n"
        "out vec4 f_color;\n"
        "out vec3 f_normal;\n"
        "void main()\n"
        "{\n"
        "    f_position = vec3(model * vec4(position, 1.0));\n"
        "    f_normal = mat3(model) * normal;"
        "    f_color = color;"
        "    gl_Position = view_projection * vec4(f_position, 1.0);\n"
        "}\n";

static const char* s_fragment_shader_text =
        "#version 410\n"
        "uniform vec3 light_position;\n"
        "uniform vec3 uniform_color;\n"
        "uniform float ambient_strength;\n"
        "in vec3 f_position;\n"
        "in vec4 f_color;\n"
        "in vec3 f_normal;\n"
        "out vec4 out_color;\n"
        "void main()\n"
        "{\n"
        "   vec3 light_color = f_color.xyz;\n"
        "   vec3 object_color = uniform_color;\n"
        "   vec3 ambient = ambient_strength * light_color;\n"
        "   vec3 normal = normalize(f_normal);\n"
        "   vec3 light_dir = normalize(light_position - f_position);\n"
        "   float diff = max(dot(normal, light_dir), 0.0);\n"
        "   vec3 diffuse = diff * light_color;\n"
        "   vec3 result = (ambient + diffuse) * object_color;\n"
        "   out_color = vec4(result, 1.0);\n"
        "}\n";

std::shared_ptr<MeshShader> DefaultShaders::mesh_shader() {
    auto shader = std::make_shared<MeshShader>(s_vertex_shader_text, s_fragment_shader_text);
    SL_VERIFY(shader->ready());
    return shader;
}

std::shared_ptr<LineShader> DefaultShaders::line_shader() {
    auto shader = std::make_shared<LineShader>(s_line_vertex_shader_text, s_line_fragment_shader_text);
    SL_VERIFY(shader->ready());
    return shader;
}

}
