#include "app/system/app.hpp"
#include "app/render/render.hpp"
#include "app/render/camera.hpp"
#include "app/scene/transform.hpp"
#include "app/scene/physics.hpp"
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

MeshPtr create_mesh_from_poly(const std::shared_ptr<ConvexPolyhedron>& geom) {
    TrimeshFactory tri_factory;
    auto trimesh = tri_factory.from_polyhedron(*geom);

    Vector<uint32_t> indices;
    Vector<Mesh::Vertex> vertices;
    for (auto& tri : trimesh->triangles()) {
        for (auto vid : tri.vert_ids) {
            indices.push_back(static_cast<uint32_t>(vertices.size()));
            auto& v = vertices.emplace_back();
            v.position = trimesh->vertices()[vid];
            v.normal = trimesh->normals()[tri.normal_id];
            v.color = Vec4{ 1.f, 0.5f, 0.5f, 0.f };
            v.tex_coords = Vec2{0.f, 0.f};
        }
    }

    return std::make_shared<Mesh>(vertices, indices);
}

class TestApp : public App {
public:
    ~TestApp() override = default;

    void on_init() override {
        m_world = std::make_unique<World>();
        m_world->add_system<PhysicsSystem>();
        m_world->add_system<CameraSystem>();
        m_world->add_system<RenderSystem>();
        m_world->create_singleton<RenderSingleton>();
        auto* physics_single = m_world->create_singleton<PhysicsSingleton>();

        physics_single->m_dynamics_world.solver().set_iteration_count(30);

        float bloat = 0.01f;
        ConvexPolyhedronFactory poly_factory;
        m_unit_box = poly_factory.box(Vec3{1.f + bloat, 1.f + bloat, 1.f + bloat}, Vec3{0.f, 0.f, 0.f});

        {
            auto visual_geom = poly_factory.box(Vec3{1.f, 1.f, 1.f}, Vec3{0.f, 0.f, 0.f});
            m_unit_box_mesh = create_mesh_from_poly(visual_geom);
        }

        m_big_box = poly_factory.box(Vec3{1.5f, 1.5f, 1.5f}, Vec3{0.f, 0.f, 0.f});
        m_big_box_mesh = create_mesh_from_poly(m_big_box);

        auto shader = std::make_shared<Shader>(s_vertex_shader_text, s_fragment_shader_text);
        SL_VERIFY(shader->ready());

        m_unit_box_material = std::make_shared<Material>(shader);
        m_unit_box_material->set_ambient_strength(0.2f);

        int mode = 2;
        if (mode == 0) {
            auto plate_geom = poly_factory.box(Vec3{20.f, 10.f, 20.f}, Vec3{0.f, 0.f, 0.f});
            auto plate_mesh = create_mesh_from_poly(plate_geom);

            auto e = m_world->create_entity();
            auto* rc = m_world->create_component<RenderComponent>(e);
            rc->mesh = plate_mesh;
            rc->material = m_unit_box_material;

            auto* tc = m_world->create_component<TransformComponent>(e);

            float angle = PI * 0.25f;
            tc->transform = Mat44::rotation({1.f, 0.f, 0.f}, angle);
            tc->transform.set_translation({0.f, -1.f, 0.f});

            auto* pc = m_world->create_component<PhysicsComponent>(e);
            pc->actor = std::make_shared<StaticActor>();
            pc->actor->set_shape<ConvexPolyhedronShape>(plate_geom);
            pc->actor->set_transform(tc->transform);

            Mat44 tr = Mat44::rotation({1.f, 0.f, 0.f}, angle);
            tr.set_translation({-5.f, 6.9f, 0.f});
            spawn_cube(tr, {}, 1.f);
        }
        else if (mode == 1) {
            int h = 2;
            float skew = 0.01f;
            for (int j = 0; j < h; j++) {
                        Mat44 tr = Mat44::rotation({0.f, 1.f, 0.f}, j * PI * 0.f);
                        tr.set_translation({skew * j, -0.011f + j * 0.991f, skew * j});
                        spawn_cube(tr, {}, 1.f);
            }

        } else if (mode == 2) {
            int h = 35;
            float spacing = 0.f;
            for (int j = 0; j < h; j++) {
                for (int i = 0; i < h - j; i++) {
                    for (int k = 0; k < 1; k++) {
                        Mat44 tr = Mat44::rotation({0.f, 1.f, 0.f}, j * PI * 0.f);
                        tr.set_translation({-float(h) / 2 + i * (1.f + spacing) + j * (0.5f + spacing/2), -0.001f + j * 0.999f, (j %2) * spacing * 0.f +  k * 1.0f});
                        spawn_cube(tr, {}, 1.f);
                    }
                }
            }
        } else if (mode == 3) {
            auto cup_geom = poly_factory.box(Vec3{20.f, 10.f, 20.f}, Vec3{0.f, 0.f, 0.f});
            Vec3 cup_pos[4] = {{-13.f, 5.f, 0.f}, {13.f, 5.f, 0.f}, {0.f, 5.f, -13.f}, {0.f, 5.f, 13.f}};

            for (int i = 0; i < 4; i++) {
                auto e = m_world->create_entity();

                auto* tc = m_world->create_component<TransformComponent>(e);
                tc->transform = Mat44::translate(cup_pos[i]);

                auto* pc = m_world->create_component<PhysicsComponent>(e);
                pc->actor = std::make_shared<StaticActor>();
                pc->actor->set_shape<ConvexPolyhedronShape>(cup_geom);
                pc->actor->set_transform(tc->transform);
            }
        }

        {
            auto floor_geom = poly_factory.box(Vec3{500.f, 1.f, 500.f}, Vec3{0.f, 0.f, 0.f});
            auto ground_mesh = create_mesh_from_poly(floor_geom);

            auto e = m_world->create_entity();
            auto* rc = m_world->create_component<RenderComponent>(e);
            rc->mesh = ground_mesh;
            rc->material = m_unit_box_material;

            auto* tc = m_world->create_component<TransformComponent>(e);
            tc->transform = Mat44::translate({0.f, -1.f, 0.f});

            auto* pc = m_world->create_component<PhysicsComponent>(e);
            pc->actor = std::make_shared<StaticActor>();
            pc->actor->set_shape<ConvexPolyhedronShape>(floor_geom);
            pc->actor->set_transform(tc->transform);
        }

        {
            auto le = m_world->create_entity();
            m_world->create_component<LightSourceComponent>(le);
            m_world->create_component<TransformComponent>(le)->transform = Mat44::translate({50.f, 100.f, 70.f});
        }

        {
            Vec3 eye = {4.f, 5.f, 10.f};
            m_cam_entity = m_world->create_entity();
            m_world->create_component<TransformComponent>(m_cam_entity)->transform = Mat44::translate(eye);
            m_world->create_component<CameraControllerComponent>(m_cam_entity);
            m_world->create_component<CameraComponent>(m_cam_entity);
        }

        set_background_color({0.2f, 0.2, 0.2f});

        m_world->get_singleton_for_write<PhysicsSingleton>()->pause = true;
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
            case Key::F:
                if (is_pressed)
                    fire_cube();
                break;
            case Key::Space:
                if (action == KeyAction::Press ) {
                    auto* phyics_single = m_world->get_singleton_for_write<PhysicsSingleton>();
                    phyics_single->pause = !phyics_single->pause;
                }
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

    void spawn_cube(const Mat44& tr, const Vec3& velocity, float mass, bool big = false) {
        auto e = m_world->create_entity();
        auto* rc = m_world->create_component<RenderComponent>(e);
        rc->mesh = big ? m_big_box_mesh : m_unit_box_mesh;
        rc->material = m_unit_box_material;

        auto* tc = m_world->create_component<TransformComponent>(e);
        tc->transform = tr;

        auto* pc = m_world->create_component<PhysicsComponent>(e);
        auto geom = big ? m_big_box : m_unit_box;

        auto actor = std::make_shared<DynamicActor>();
        actor->set_shape<ConvexPolyhedronShape>(geom);
        actor->set_transform(tc->transform);
        actor->body().set_velocity(velocity);
        actor->body().set_mass(mass);
        float inertia = mass / 6.f;
        actor->body().set_local_inertia({inertia, inertia, inertia});

        pc->actor = std::move(actor);
    }

    void fire_cube() {
        auto* cam_tr = m_world->get_component<TransformComponent>(m_cam_entity);
        auto vel = cam_tr->transform.apply_normal({0.f, 0.f, -30.f});
        spawn_cube(cam_tr->transform, vel, 5.f, true);
    }

    MeshPtr m_big_box_mesh;
    std::shared_ptr<ConvexPolyhedron> m_big_box;

    MaterialPtr m_unit_box_material;
    MeshPtr m_unit_box_mesh;
    std::shared_ptr<ConvexPolyhedron> m_unit_box;

    bool m_cam_move_mode = false;
    Entity m_cam_entity;
    std::unique_ptr<World> m_world;
};

int main() {
    AppManager::run<TestApp>({1600, 1200, "Hello world"});
    return 0;
}
