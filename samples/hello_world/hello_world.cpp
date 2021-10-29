#include "app/system/app.hpp"
#include "app/render/render.hpp"
#include "app/render/camera.hpp"
#include "app/scene/transform.hpp"
#include "app/ecs/world.hpp"
#include "slope/debug/assert.hpp"
#include "slope/debug/log.hpp"
#include "slope/collision/geometry.hpp"
#include <memory>

using namespace slope;
using namespace slope::app;

static const char* s_vertex_shader_text =
        "#version 410\n"
        "layout (location = 0) in vec3 position;\n"
        "layout (location = 1) in vec3 normal;\n"
        "layout (location = 2) in vec4 color;\n"
        "layout (location = 3) in vec2 tex_coords;\n"
        "layout (location = 4) in mat4 model;\n"
        "uniform mat4 view_projection;\n"
        "out vec3 f_position;\n"
        "out vec3 f_normal;\n"
        "void main()\n"
        "{\n"
        "    f_position = vec3(model * vec4(position, 1.0));\n"
        "    f_normal = mat3(model) * normal;"
        "    gl_Position = view_projection * vec4(f_position, 1.0);\n"
        "}\n";

static const char* s_fragment_shader_text =
        "#version 410\n"
        "uniform vec3 light_position;\n"
        "uniform float ambient_strength;\n"
        "in vec3 f_position;\n"
        "in vec3 f_normal;\n"
        "out vec4 out_color;\n"
        "void main()\n"
        "{\n"
        "   vec3 light_color = vec3(1.0, 1.0, 1.0);\n"
        "   vec3 object_color = vec3(0.9, 0.8, 0.0);\n"
        "   vec3 ambient = ambient_strength * light_color;\n"
        "   vec3 normal = normalize(f_normal);\n"
        "   vec3 light_dir = normalize(light_position - f_position);\n"
        "   float diff = max(dot(normal, light_dir), 0.0);\n"
        "   vec3 diffuse = diff * light_color;\n"
        "   vec3 result = (ambient + diffuse) * object_color;\n"
        "   out_color = vec4(result, 1.0);\n"
        "}\n";

class TestApp : public App {
public:
    ~TestApp() override = default;

    void on_init() override {
        ConvexPolyhedronFactory poly_factory;
        poly_factory.box(Vec3{1.f, 1.f, 1.f}, Vec3{0.f, 0.f, 0.f});

        TrimeshFactory tri_factory;
        tri_factory.from_polyhedron(poly_factory.result());
        auto trimesh = tri_factory.result();

        Vector<uint32_t> indices;
        Vector<Mesh::Vertex> vertices;
        for (auto& tri : trimesh.triangles()) {
            for (auto vid : tri.vert_ids) {
                indices.push_back(static_cast<uint32_t>(vertices.size()));
                auto& v = vertices.emplace_back();
                v.position = trimesh.vertices()[vid];
                v.normal = trimesh.normals()[tri.normal_id];
                v.color = Vec4{ 1.f, 0.5f, 0.5f, 0.f };
                v.tex_coords = Vec2{0.f, 0.f};
            }
        }

        m_world = std::make_unique<World>();
        m_world->add_system<CameraSystem>();
        m_world->add_system<RenderSystem>();

        m_world->create_singleton<RenderSingleton>();

        auto mesh = std::make_shared<Mesh>(vertices, indices);

        auto shader = std::make_shared<Shader>(s_vertex_shader_text, s_fragment_shader_text);
        SL_VERIFY(shader->ready());

        auto material = std::make_shared<Material>(shader);
        material->set_ambient_strength(0.15f);

        for (int i = 0; i < 1; i++) {

            auto e = m_world->create_entity();
            auto* rc = m_world->create_component<RenderComponent>(e);
            rc->mesh = mesh;
            rc->material = material;

            auto* tc = m_world->create_component<TransformComponent>(e);
            tc->transform = Mat44::translation({0.f, 0.f, 0.f});
        }

        auto le = m_world->create_entity();
        m_world->create_component<LightSourceComponent>(le);
        m_world->create_component<TransformComponent>(le)->transform = Mat44::translation({50.f, 100.f, 70.f});

        Vec3 eye = {0.f, 0.f, 2.f};

        m_cam_entity = m_world->create_entity();
        m_world->create_component<TransformComponent>(m_cam_entity)->transform = Mat44::translation(eye);
        m_world->create_component<CameraControllerComponent>(m_cam_entity);
        m_world->create_component<CameraComponent>(m_cam_entity);

        set_background_color({0.1f, 0.6f, 0.7f});
    }

    void update(float dt) override {
        m_world->update(dt);
    }

    void on_window_resize(int width, int height) override {
        auto& cam = m_world->get_component_for_write<CameraComponent>(m_cam_entity)->camera;
        cam.set_aspect_ratio(static_cast<float>(width) / height);
    }

    void on_key(Key key, int scancode, KeyAction action, int mods) override {
        bool is_pressed = action == KeyAction::Press || action == KeyAction::Repeat;

        auto* cam_ctl = m_world->get_component_for_write<CameraControllerComponent>(m_cam_entity);

        switch (key) {
            case Key::A:
                cam_ctl->move_left = is_pressed;
                break;
            case Key::D:
                cam_ctl->move_right = is_pressed;
                break;
            case Key::W:
                cam_ctl->move_fwd = is_pressed;
                break;
            case Key::S:
                cam_ctl->move_bkwd = is_pressed;
                break;
            default:
                break;
        }
    }

    void on_cursor_move(double x_delta, double y_delta) override {
        if (m_cam_move_mode) {
            auto* cam_ctl = m_world->get_component_for_write<CameraControllerComponent>(m_cam_entity);
            cam_ctl->rotate(x_delta, -y_delta);

        }
    }

    void on_mouse_button(MouseButton button, KeyAction action, KeyMod::Raw mods) override {
        if (button == MouseButton::B_1) {
            m_cam_move_mode = (action == KeyAction::Press);
        }
    }

    bool m_cam_move_mode = false;
    Entity m_cam_entity;
    std::unique_ptr<World> m_world;
};

int main() {
    AppManager::run<TestApp>({1024, 768, "Hello world"});
    return 0;
}
