#include "app/system/app.hpp"
#include "app/render/mesh.hpp"
#include "app/render/material.hpp"
#include "app/render/render.hpp"
#include "app/render/shader_program.hpp"
#include "slope/debug/assert.hpp"
#include "slope/debug/log.hpp"
#include <memory>

#include "app/render/render_system.hpp"
#include "app/ecs/world.hpp"

using namespace slope;
using namespace slope::app;

static const char* s_vertex_shader_text =
        "#version 410\n"
        "layout (location = 0) in vec3 position;\n"
        "layout (location = 1) in vec3 normal;\n"
        "layout (location = 2) in vec4 color;\n"
        "layout (location = 3) in vec2 tex_coords;\n"
        "layout (location = 4) in mat4 MVP;\n"
        "uniform mat4 uMVP;\n"
        "out vec3 v_color;\n"
        "void main()\n"
        "{\n"
        "    gl_Position = MVP * vec4(position, 1.0);\n"
        "    v_color = color.xyz;\n"
        "}\n";

static const char* s_fragment_shader_text =
        "#version 410\n"
        "in vec3 v_color;\n"
        "out vec4 fragment;\n"
        "void main()\n"
        "{\n"
        "    fragment = vec4(v_color, 1.0);\n"
        "}\n";

static const Vector<Mesh::Vertex> s_vertices = {
        { Vec3{ -0.6f, -0.4f, 0.f }, Vec3{ 0.f, 0.f, 1.f }, Vec4{ 1.f, 0.f, 0.f, 0.5f }, Vec2{0.f, 0.f} },
        { Vec3{  0.6f, -0.4f, 0.f }, Vec3{ 0.f, 0.f, 1.f }, Vec4{0.f, 1.f, 0.f, 0.5f}, Vec2{0.f, 0.f}  },
        { Vec3{   0.f,  0.6f, 0.f }, Vec3{ 0.f, 0.f, 1.f }, Vec4{0.f, 0.f, 1.f, 0.5f}, Vec2{0.f, 0.f}  }
};

static const Vector<uint32_t> s_indices = {0, 1, 2};

class TestApp : public App {
public:

    TestApp(const AppCfg& cfg) : App(cfg) {

        m_world = std::make_unique<World>();
        m_world->add_system<RenderSystem>();

        auto* render_single = m_world->create_singleton<RenderSingleton>();

        auto mesh = std::make_shared<Mesh>(s_vertices, s_indices, render_single->render.instancing_buffer());

        auto shader = std::make_shared<ShaderProgram>(s_vertex_shader_text, s_fragment_shader_text);
        SL_VERIFY(shader->ready());

        auto material = std::make_shared<Material>(shader);

        for (int i = 0; i < 10000; i++) {

            auto e = m_world->create_entity();
            auto* rc = m_world->create_component<RenderComponent>(e);
            rc->mesh = mesh;
            rc->offs.x = i * 0.0002f;

            rc->material = material;
        }
    }

    void update(float dt) override {
        m_world->update(dt);
    }

    std::unique_ptr<World> m_world;
};

int main() {
    TestApp({640, 480, "Hello world"}).run();
    return 0;
}
