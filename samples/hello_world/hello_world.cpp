#include "app/system/app.hpp"
#include "app/render/render_system.hpp"
#include "app/render/camera.hpp"
#include "app/render/default_shaders.hpp"
#include "app/render/debug_drawer_impl.hpp"
#include "app/scene/transform.hpp"
#include "app/scene/physics_system.hpp"
#include "app/scene/ui_overlay_system.hpp"
#include "app/ecs/world.hpp"
#include "slope/debug/log.hpp"
#include "slope/collision/geometry.hpp"
#include "slope/collision/gjk.hpp"
#include <memory>

using namespace slope;
using namespace slope::app;

static constexpr float BLOAT = 0.01f;

static constexpr float SPHERE_RADIUS = 1.f;
static constexpr float SPHERE_MASS = 1.f;
static constexpr float SPHERE_SPEED = 15.f;

auto create_mesh_from_poly(const std::shared_ptr<ConvexPolyhedron>& geom) {
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
            v.color = Vec4{ 1.f, 1.f, 1.f, 0.f };
            v.tex_coords = Vec2{0.f, 0.f};
        }
    }

    return std::make_shared<Mesh>(vertices, indices);
}

auto create_sphere_mesh(float radius) {
    constexpr int YN = 30;
    constexpr int PN = 15;

    auto up = Vec3{0.f, 1.f, 0.f};
    auto side = Vec3{1.f, 0.f, 0.f};

    Vector<uint32_t> indices;
    Vector<Mesh::Vertex> vertices;

    float prev_pitch = -slope::PI * 0.5f;

    for (int j = 1; j <= PN; j++) {
        float pitch = j * slope::PI / PN - slope::PI * 0.5f;

        float prev_yaw = 0.f;
        for (int i = 1; i <= YN; i++) {
            float yaw = i * 2.f * slope::PI / YN;

            auto ur = static_cast<uint32_t>(vertices.size());
            auto ul = ur + 1;
            auto dr = ur + 2;
            auto dl = ur + 3;

            Vec4 color = pitch > 0 ? Vec4{0.6, 0.8, 0.2, 0.f} : Vec4{0.8, 0.6, 0.2, 0.f};
            if (yaw > slope::PI)
                color.z = 0.7;

            auto& vur = vertices.emplace_back();
            vur.position = (Mat44::rotation(side, pitch) * Mat44::rotation(up, yaw)).apply_normal({0.f, 0.f, radius});
            vur.normal = vur.position.normalized();
            vur.color = color;
            vur.tex_coords = Vec2{0.f, 0.f};

            auto& vul = vertices.emplace_back();
            vul.position = (Mat44::rotation(side, pitch) * Mat44::rotation(up, prev_yaw)).apply_normal({0.f, 0.f, radius});
            vul.normal =  vul.position.normalized();
            vul.color = color;
            vul.tex_coords = Vec2{0.f, 0.f};

            auto& vdr = vertices.emplace_back();
            vdr.position = (Mat44::rotation(side, prev_pitch) * Mat44::rotation(up, yaw)).apply_normal({0.f, 0.f, radius});
            vdr.normal =  vdr.position.normalized();
            vdr.color = color;
            vdr.tex_coords = Vec2{0.f, 0.f};

            auto& vdl = vertices.emplace_back();
            vdl.position = (Mat44::rotation(side, prev_pitch) * Mat44::rotation(up, prev_yaw)).apply_normal({0.f, 0.f, radius});
            vdl.normal =  vdl.position.normalized();
            vdl.color = color;
            vdl.tex_coords = Vec2{0.f, 0.f};

            indices.push_back(ur);
            indices.push_back(dr);
            indices.push_back(ul);

            indices.push_back(dr);
            indices.push_back(ul);
            indices.push_back(dl);

            prev_yaw = yaw;
        }

        prev_pitch = pitch;
    }

    return std::make_shared<Mesh>(vertices, indices);

    //return create_mesh_from_poly(ConvexPolyhedronFactory().box({1, 1, 1}));
}

class TestApp : public App {
public:
    ~TestApp() override = default;

    void on_init() override {
        m_world = std::make_unique<World>();
        m_world->add_system<PhysicsSystem>();
        m_world->add_system<CameraSystem>();
        m_world->add_system<RenderSystem>();
        m_world->add_system<UIOverlaySystem>();
        m_world->create_singleton<RenderSingleton>();
        auto* physics_single = m_world->create_singleton<PhysicsSingleton>();

        auto* debug_draw = m_world->create<DebugDrawComponent>(m_world->create_entity());
        debug_draw->drawer = std::make_shared<DebugDrawerImpl>();

        physics_single->dynamics_world.set_debug_drawer(debug_draw->drawer);
        physics_single->dynamics_world.config().solver_config.iteration_count = 30;

        ConvexPolyhedronFactory poly_factory;
        m_unit_box = poly_factory.box(Vec3{1.f + BLOAT, 1.f + BLOAT, 1.f + BLOAT}, Vec3{0.f, 0.f, 0.f});

        {
            auto visual_geom = poly_factory.box(Vec3{1.f, 1.f, 1.f}, Vec3{0.f, 0.f, 0.f});
            m_unit_box_mesh = create_mesh_from_poly(visual_geom);
        }

        m_big_box = poly_factory.box(Vec3{1.5f, 1.5f, 1.5f}, Vec3{0.f, 0.f, 0.f});
        m_big_box_mesh = create_mesh_from_poly(m_big_box);

        m_unit_box_material = std::make_shared<Material>(DefaultShaders::mesh_shader());
        m_unit_box_material->set_ambient_strength(0.2f);
        m_unit_box_material->set_color({0.9, 0.75, 0.0});

        m_sphere_mesh = create_sphere_mesh(SPHERE_RADIUS);
        m_sphere_material = std::make_shared<Material>(DefaultShaders::mesh_shader());
        m_sphere_material->set_ambient_strength(0.2f);
        //m_sphere_material->set_color({0.5, 0.8, 0.1});

        int mode = 2;
        if (mode == 0) {
            auto plate_geom = poly_factory.box(Vec3{20.f, 10.f, 20.f}, Vec3{0.f, 0.f, 0.f});
            auto plate_mesh = create_mesh_from_poly(plate_geom);

            auto e = m_world->create_entity();
            auto* rc = m_world->create<RenderComponent>(e);
            rc->mesh = plate_mesh;
            rc->material = m_unit_box_material;

            auto* tc = m_world->create<TransformComponent>(e);

            float angle = PI * 0.25f;
            tc->transform = Mat44::rotation({1.f, 0.f, 0.f}, angle);
            tc->transform.set_translation({0.f, -1.f, 0.f});

            auto* pc = m_world->create<PhysicsComponent>(e);
            pc->actor = std::make_shared<StaticActor>();
            pc->actor->set_shape<ConvexPolyhedronShape>(plate_geom);
            pc->actor->set_transform(tc->transform);

            Mat44 tr = Mat44::rotation({1.f, 0.f, 0.f}, angle);
            tr.set_translation({-5.f, 6.9f, 0.f});
            spawn_cube(tr, {}, 1.f);
        }
        else if (mode == 1) {
            int h = 5;
            float skew = 0.1f;
            for (int j = 0; j < h; j++) {
                        Mat44 tr = Mat44::rotation({0.f, 1.f, 0.f}, j * PI * 0.f);
                        tr.set_translation({skew * j, 0.5f + j * 2.02f, skew * j});
                        spawn_sphere(tr, {}, 1.f);
            }

        } else if (mode == 2) {
            auto rot = Mat44::rotation({0.f, 1.f, 0.f}, 0.5f);
            int h = 35;
            float spacing = 0.f;
            for (int j = 0; j < h; j++) {
                for (int i = 0; i < h - j; i++) {
                    for (int k = 0; k < 1; k++) {
                        Mat44 tr = Mat44::rotation({0.f, 1.f, 0.f}, j * PI * 0.f);
                        tr.set_translation({-float(h) / 2 + i * (1.f + spacing) + j * (0.5f + spacing/2), -0.001f + j * 0.999f, (j %2) * spacing * 0.f +  k * 1.0f});
                        //tr *= rot;
                        spawn_cube(tr, {}, 1.f);
                    }
                }
            }
        } else if (mode == 3) {
            auto cup_geom = poly_factory.box(Vec3{20.f, 10.f, 20.f}, Vec3{0.f, 0.f, 0.f});
            Vec3 cup_pos[4] = {{-13.f, 5.f, 0.f}, {13.f, 5.f, 0.f}, {0.f, 5.f, -13.f}, {0.f, 5.f, 13.f}};

            for (int i = 0; i < 4; i++) {
                auto e = m_world->create_entity();

                auto* tc = m_world->create<TransformComponent>(e);
                tc->transform = Mat44::translate(cup_pos[i]);

                auto* pc = m_world->create<PhysicsComponent>(e);
                pc->actor = std::make_shared<StaticActor>();
                pc->actor->set_shape<ConvexPolyhedronShape>(cup_geom);
                pc->actor->set_transform(tc->transform);
            }
        }

        {
            float floor_size = 500.f;
            auto floor_geom = poly_factory.box(Vec3{floor_size, 1.f, floor_size}, Vec3{0.f, 0.f, 0.f});
            auto ground_mesh = create_mesh_from_poly(floor_geom);

            auto e = m_world->create_entity();
            auto* rc = m_world->create<RenderComponent>(e);
            rc->mesh = ground_mesh;

            rc->material = std::make_shared<Material>(DefaultShaders::mesh_shader());
            rc->material->set_ambient_strength(0.1f);
            rc->material->set_color({0.8, 0.8, 0.9});

            auto* tc = m_world->create<TransformComponent>(e);
            tc->transform = Mat44::translate({0.f, -1.f, 0.f});

            auto* pc = m_world->create<PhysicsComponent>(e);
            pc->actor = std::make_shared<StaticActor>();
            pc->actor->set_shape<ConvexPolyhedronShape>(floor_geom);
            pc->actor->set_friction(0.7f);
            pc->actor->set_transform(tc->transform);
        }

        {
            auto le = m_world->create_entity();
            m_world->create<LightSourceComponent>(le);
            m_world->create<TransformComponent>(le)->transform = Mat44::translate({50.f, 100.f, 70.f});
        }

        {
            Vec3 eye = {4.f, 5.f, 10.f};
            m_cam_entity = m_world->create_entity();
            m_world->create<TransformComponent>(m_cam_entity)->transform = Mat44::translate(eye);
            m_world->create<CameraControllerComponent>(m_cam_entity);
            m_world->create<CameraComponent>(m_cam_entity);
        }

        init_gjk_shape();

        set_background_color({0.2f, 0.2, 0.2f});

        m_world->modify_singleton<PhysicsSingleton>()->pause = true;
    }

    void init_gjk_shape() {
        ConvexPolyhedronFactory poly_factory;

        auto gjk_visual_box = poly_factory.box(Vec3{1.f + BLOAT, 1.f + BLOAT, 1.f + BLOAT}, Vec3{0.f, 0.f, 0.f});

        auto gjk_box = poly_factory.box(Vec3{1.f + BLOAT, 1.f + BLOAT, 1.f + BLOAT}, Vec3{0.f, 0.f, 0.f});
        m_gjk_shape = std::make_unique<ConvexPolyhedronShape>(gjk_box);

        m_gjk_entity = m_world->create_entity();
        auto* tc = m_world->create<TransformComponent>(m_gjk_entity);
        tc->transform.set_translation({10.f, -15.f, 10.f});

        m_gjk_shape->set_transform(tc->transform);

        auto* rc = m_world->create<RenderComponent>(m_gjk_entity);
        rc->mesh = create_mesh_from_poly(gjk_visual_box);
        //rc->mesh = create_sphere_mesh(1.f);
        rc->material = std::make_shared<Material>(DefaultShaders::mesh_shader());
        rc->material->set_ambient_strength(0.5f);
        rc->material->set_color({0.4, 0.8, 0.4});
    }

    void collide_gjk() {

        if (m_bind_gjk_to_camera) {
            auto* cam_tc = m_world->get<TransformComponent>(m_cam_entity);
            auto* gjk_tc = m_world->modify<TransformComponent>(m_gjk_entity);
            gjk_tc->transform = Mat44::translate({0.f, 0.f, -0.4f}) * cam_tc->transform;
            m_gjk_shape->set_transform(gjk_tc->transform);
        }

        bool has_collision = false;



        for (auto e : m_world->view<PhysicsComponent>()) {
            auto* pc = m_world->get<PhysicsComponent>(e);
            auto* shape2 = static_cast<ConvexPolyhedronShape*>(&pc->actor->shape());
            if (m_gjk_solver.intersect(&*m_gjk_shape, shape2)) {
                has_collision = true;
                break;
            }
        }

        auto* gjk_rc = m_world->get<RenderComponent>(m_gjk_entity);

        if (has_collision)
            gjk_rc->material->set_color({0.8, 0.4, 0.4});
        else
            gjk_rc->material->set_color({0.4, 0.8, 0.4});
    }

    void update(float dt) override {
        m_world->update(dt);
        //collide_gjk();
    }

    void on_window_resize(int width, int height) override {
        auto& cam = m_world->modify<CameraComponent>(m_cam_entity)->camera;
        cam.set_aspect_ratio(static_cast<float>(width) / height);
    }

    void on_key(Key key, int scancode, KeyAction action, int mods) override {
        bool is_pressed = action == KeyAction::Press || action == KeyAction::Repeat;

        auto* cam_ctl = m_world->modify<CameraControllerComponent>(m_cam_entity);

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

            case Key::C:
                if (action == KeyAction::Press) {
                    m_bind_gjk_to_camera = !m_bind_gjk_to_camera;
                    if (m_bind_gjk_to_camera) {
                        auto* cam_tc = m_world->modify<TransformComponent>(m_cam_entity);
                        auto* gjk_tc = m_world->get<TransformComponent>(m_gjk_entity);
                        cam_tc->transform = gjk_tc->transform;
                    }
                    break;
                }

            case Key::LeftShift: {
                auto* cam = m_world->modify<CameraControllerComponent>(m_cam_entity);
                cam->velocity = is_pressed ? 1.f : 8.f;
                break;
            }

            case Key::F:
                    if (is_pressed)
                        fire_cube();
                    break;

            case Key::G:
                if (is_pressed)
                    fire_sphere();
                break;

            case Key::Space:
                if (action == KeyAction::Press ) {
                    auto* phyics_single = m_world->modify_singleton<PhysicsSingleton>();
                    phyics_single->pause = !phyics_single->pause;
                }
                break;
            default:
                break;
        }
    }

    void on_cursor_move(double x_delta, double y_delta) override {
        if (m_cam_move_mode) {
            auto* cam_ctl = m_world->modify<CameraControllerComponent>(m_cam_entity);
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
        auto* rc = m_world->create<RenderComponent>(e);
        rc->mesh = big ? m_big_box_mesh : m_unit_box_mesh;
        rc->material = m_unit_box_material;

        auto* tc = m_world->create<TransformComponent>(e);
        tc->transform = tr;

        auto* pc = m_world->create<PhysicsComponent>(e);
        auto geom = big ? m_big_box : m_unit_box;

        auto actor = std::make_shared<DynamicActor>();
        actor->set_shape<ConvexPolyhedronShape>(geom);
        actor->set_transform(tc->transform);
        actor->body().set_velocity(velocity);
        actor->body().set_mass(mass);
        float inertia = mass / 6.f;
        actor->body().set_local_inertia({inertia, inertia, inertia});

        actor->set_friction(0.7f);

        pc->actor = std::move(actor);
    }

    void spawn_sphere(const Mat44& tr, const Vec3& velocity, float mass) {
        auto e = m_world->create_entity();
        auto* rc = m_world->create<RenderComponent>(e);
        rc->mesh = m_sphere_mesh;
        rc->material = m_sphere_material;

        auto* tc = m_world->create<TransformComponent>(e);
        tc->transform = tr;

        auto* pc = m_world->create<PhysicsComponent>(e);

        float radius = SPHERE_RADIUS + BLOAT;

        auto actor = std::make_shared<DynamicActor>();
        actor->set_shape<SphereShape>(radius);
        actor->set_transform(tc->transform);
        actor->body().set_velocity(velocity);
        actor->body().set_mass(mass);
        float inertia = 0.4f * mass * radius * radius;
        actor->body().set_local_inertia({inertia, inertia, inertia});

        actor->set_friction(0.7f);

        pc->actor = std::move(actor);
    }

    void fire_cube() {
        auto* cam_tr = m_world->get<TransformComponent>(m_cam_entity);
        auto vel = cam_tr->transform.apply_normal({0.f, 0.f, -25.f});
        spawn_cube(cam_tr->transform, vel, 5.f, true);
    }

    void fire_sphere() {
        auto* cam_tr = m_world->get<TransformComponent>(m_cam_entity);
        auto vel = cam_tr->transform.apply_normal({0.f, 0.f, -SPHERE_SPEED});
        spawn_sphere(cam_tr->transform, vel, SPHERE_MASS);
    }

    std::shared_ptr<Mesh> m_big_box_mesh;
    std::shared_ptr<ConvexPolyhedron> m_big_box;

    std::shared_ptr<Material> m_unit_box_material;
    std::shared_ptr<Mesh> m_unit_box_mesh;
    std::shared_ptr<ConvexPolyhedron> m_unit_box;

    std::shared_ptr<Mesh> m_sphere_mesh;
    std::shared_ptr<Material> m_sphere_material;

    GJKSolver m_gjk_solver;
    Entity m_gjk_entity;
    std::shared_ptr<Mesh> m_gjk_mesh;
    std::unique_ptr<ConvexPolyhedronShape> m_gjk_shape;
    bool m_bind_gjk_to_camera = false;

    bool m_cam_move_mode = false;
    Entity m_cam_entity;
    std::unique_ptr<World> m_world;
};

int main() {
    AppManager::run<TestApp>({1600, 1200, "Hello world"});
    return 0;
}
