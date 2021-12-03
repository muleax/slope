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
#include "slope/collision/narrowphase/narrowphase.hpp"
#include "slope/collision/narrowphase/default_narrowphase_backends.hpp"
#include "slope/collision/shape/sphere_shape.hpp"
#include "slope/collision/shape/capsule_shape.hpp"
#include "slope/collision/shape/convex_polyhedron_shape.hpp"
#include "imgui/imgui.h"
#include <memory>

using namespace slope;
using namespace slope::app;

static constexpr float BLOAT = 0.01f;

static constexpr float SPHERE_RADIUS = 1.f;
static constexpr float SPHERE_MASS = 1.f;
static constexpr float SPHERE_SPEED = 15.f;

static constexpr float CAPSULE_RADIUS = 1.f;
static constexpr float CAPSULE_HEIGHT = 2.f;
static constexpr float CAPSULE_MASS = 1.f;
static constexpr float CAPSULE_SPEED = 20.f;


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
}

auto create_capsule_mesh(float radius, float axis_length) {
    constexpr int YN = 30;
    constexpr int PN = 16;

    auto up = Vec3{0.f, 1.f, 0.f};
    auto side = Vec3{1.f, 0.f, 0.f};

    Vector<uint32_t> indices;
    Vector<Mesh::Vertex> vertices;

    float prev_pitch = -slope::PI * 0.5f;

    for (int j = 1; j <= PN; j++) {
        float pitch = j * slope::PI / PN - slope::PI * 0.5f;

        Vec3 hemisphere_offset = {0.f, axis_length * (j <= PN / 2 ? 0.5f : -0.5f), 0.f};

        float prev_yaw = 0.f;
        for (int i = 1; i <= YN; i++) {
            float yaw = i * 2.f * slope::PI / YN;

            auto ur = static_cast<uint32_t>(vertices.size());
            auto ul = ur + 1;
            auto dr = ur + 2;
            auto dl = ur + 3;

            Vec4 color = (yaw > slope::PI) ? Vec4{0.6, 0.8, 0.2, 0.f} : Vec4{0.8, 0.6, 0.2, 0.f};

            auto& vur = vertices.emplace_back();
            vur.position = (Mat44::rotation(side, pitch) * Mat44::rotation(up, yaw)).apply_normal({0.f, 0.f, radius});
            vur.normal = vur.position.normalized();
            vur.position += hemisphere_offset;
            vur.color = color;
            vur.tex_coords = Vec2{0.f, 0.f};

            auto& vul = vertices.emplace_back();
            vul.position = (Mat44::rotation(side, pitch) * Mat44::rotation(up, prev_yaw)).apply_normal({0.f, 0.f, radius});
            vul.normal =  vul.position.normalized();
            vul.position += hemisphere_offset;
            vul.color = color;
            vul.tex_coords = Vec2{0.f, 0.f};

            auto& vdr = vertices.emplace_back();
            vdr.position = (Mat44::rotation(side, prev_pitch) * Mat44::rotation(up, yaw)).apply_normal({0.f, 0.f, radius});
            vdr.normal =  vdr.position.normalized();
            vdr.position += hemisphere_offset;
            vdr.color = color;
            vdr.tex_coords = Vec2{0.f, 0.f};

            auto& vdl = vertices.emplace_back();
            vdl.position = (Mat44::rotation(side, prev_pitch) * Mat44::rotation(up, prev_yaw)).apply_normal({0.f, 0.f, radius});
            vdl.normal =  vdl.position.normalized();
            vdl.position += hemisphere_offset;
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

    float prev_yaw = 0.f;
    for (int i = 1; i <= YN; i++) {
        float yaw = i * 2.f * slope::PI / YN;

        Vec3 hemisphere_offset = {0.f, axis_length * 0.5f, 0.f};

        auto ur = static_cast<uint32_t>(vertices.size());
        auto ul = ur + 1;
        auto dr = ur + 2;
        auto dl = ur + 3;

        Vec4 color = (yaw > slope::PI) ? Vec4{0.6, 0.8, 0.2, 0.f} : Vec4{0.8, 0.6, 0.2, 0.f};

        auto& vur = vertices.emplace_back();
        vur.position = Mat44::rotation(up, yaw).apply_normal({0.f, 0.f, radius}) + hemisphere_offset;
        vur.normal = Vec3{vur.position.x, 0.f, vur.position.z}.normalized();
        vur.color = color;
        vur.tex_coords = Vec2{0.f, 0.f};

        auto& vul = vertices.emplace_back();
        vul.position = Mat44::rotation(up, prev_yaw).apply_normal({0.f, 0.f, radius}) + hemisphere_offset;
        vul.normal = Vec3{vul.position.x, 0.f, vul.position.z}.normalized();
        vul.color = color;
        vul.tex_coords = Vec2{0.f, 0.f};

        auto& vdr = vertices.emplace_back();
        vdr.position = Mat44::rotation(up, yaw).apply_normal({0.f, 0.f, radius}) - hemisphere_offset;
        vdr.normal = Vec3{vdr.position.x, 0.f, vdr.position.z}.normalized();
        vdr.color = color;
        vdr.tex_coords = Vec2{0.f, 0.f};

        auto& vdl = vertices.emplace_back();
        vdl.position = Mat44::rotation(up, prev_yaw).apply_normal({0.f, 0.f, radius}) - hemisphere_offset;
        vdl.normal = Vec3{vdl.position.x, 0.f, vdl.position.z}.normalized();
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
        m_world->add_system<UIOverlaySystem>();
        m_world->create_singleton<RenderSingleton>();
        auto* physics_single = m_world->create_singleton<PhysicsSingleton>();

        auto* debug_draw = m_world->create<DebugDrawComponent>(m_world->create_entity());
        debug_draw->drawer = std::make_shared<DebugDrawerImpl>();

        physics_single->dynamics_world.set_debug_drawer(debug_draw->drawer);
        physics_single->dynamics_world.config().solver_config.iteration_count = 30;
        //physics_single->dynamics_world.config().gravity.set_zero();
        //physics_single->time_step /= 5.f;

        ConvexPolyhedronFactory poly_factory;
        m_unit_box = poly_factory.box(Vec3{1.f + BLOAT, 1.f + BLOAT, 1.f + BLOAT}, Vec3{0.f, 0.f, 0.f});
        auto visual_geom = poly_factory.box(Vec3{1.f, 1.f, 1.f}, Vec3{0.f, 0.f, 0.f});
        m_unit_box_mesh = create_mesh_from_poly(visual_geom);

        m_big_box = poly_factory.box(Vec3{1.5f, 1.5f, 1.5f}, Vec3{0.f, 0.f, 0.f});
        m_big_box_mesh = create_mesh_from_poly(m_big_box);

        m_unit_box_material = std::make_shared<Material>(DefaultShaders::mesh_shader());
        m_unit_box_material->set_ambient_strength(0.2f);
        m_unit_box_material->set_color({0.9, 0.75, 0.0});

        m_sphere_mesh = create_sphere_mesh(SPHERE_RADIUS);
        m_sphere_material = std::make_shared<Material>(DefaultShaders::mesh_shader());
        m_sphere_material->set_ambient_strength(0.2f);

        m_capsule_mesh = create_capsule_mesh(CAPSULE_RADIUS, CAPSULE_HEIGHT);
        m_capsule_material = std::make_shared<Material>(DefaultShaders::mesh_shader());
        m_capsule_material->set_ambient_strength(0.2f);

        m_narrowphase.add_backend<GJKConvexPolyhedronBackend>();
        m_narrowphase.add_backend<ConvexPolyhedronSphereBackend>();
        m_narrowphase.add_backend<ConvexPolyhedronCapsuleBackend>();
        m_narrowphase.add_backend<CapsuleSphereBackend>();
        m_narrowphase.add_backend<CapsuleBackend>();
        m_narrowphase.add_backend<SphereBackend>();

        if (m_mode == 0) {
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
        else if (m_mode == 1) {
            int h = 5;
            float skew = 0.1f;
            for (int j = 0; j < h; j++) {
                        Mat44 tr = Mat44::rotation({0.f, 1.f, 0.f}, j * PI * 0.f);
                        tr.set_translation({skew * j, 0.5f + j * 2.02f, skew * j});
                        spawn_sphere(tr, {}, 1.f);
            }

        } else if (m_mode == 2) {
            auto rot = Mat44::rotation({0.f, 1.f, 0.f}, 0.5f);
            int h = 25;
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
        } else if (m_mode == 3) {
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
        } else if (m_mode == 4) {
            physics_single->dynamics_world.config().gravity.set_zero();

            auto box = poly_factory.box(Vec3{2.f, 4.f, 0.4f }, Vec3{0.f, 0.f, 0.f});

            auto e = m_world->create_entity();
            auto* rc = m_world->create<RenderComponent>(e);
            rc->mesh = create_mesh_from_poly(box);
            rc->material = m_unit_box_material;

            auto* tc = m_world->create<TransformComponent>(e);
            tc->transform = Mat44::translate({0.f, 5.f, 0.f});

            auto* pc = m_world->create<PhysicsComponent>(e);

            auto actor = std::make_shared<DynamicActor>();
            actor->set_shape<ConvexPolyhedronShape>(box);
            actor->set_transform(tc->transform);
            actor->body().set_velocity(Vec3::zero());
            //actor->body().set_ang_velocity({8.f, 5.830945f, 1.f});
            actor->body().set_ang_velocity({8.f, 5.85f, 1.f});
            actor->body().set_mass(1.f);

            Vec3 localInertia = {1.666666f, 0.666667f, 5.666666f};
            actor->body().set_local_inertia(localInertia);

            actor->set_friction(0.5f);

            gyro_actor = actor.get();
            pc->actor = std::move(actor);

        } else if (m_mode == 5) {

            physics_single->dynamics_world.config().gravity.set_zero();
            physics_single->dynamics_world.config().disable_constraint_resolving = true;
            physics_single->dynamics_world.config().delay_integration = true;

            auto box = poly_factory.box(Vec3{2.f, 2.f, 2.f}, Vec3{0.f, 0.f, 0.f});

            auto e = m_world->create_entity();
            auto* rc = m_world->create<RenderComponent>(e);
            rc->mesh = create_mesh_from_poly(box);
            rc->material = m_unit_box_material;
            auto* tc = m_world->create<TransformComponent>(e);
            tc->transform = Mat44::translate({0.f, 5.f, 0.f});

            auto* pc = m_world->create<PhysicsComponent>(e);
            auto actor = std::make_shared<DynamicActor>();
            actor->set_shape<ConvexPolyhedronShape>(box);
            actor->set_transform(tc->transform);
            pc->actor = std::move(actor);

            auto box2 = poly_factory.box(Vec3{1.f, 1.f, 1.f}, Vec3{0.f, 0.f, 0.f});

            auto se = m_world->create_entity();
            auto* src = m_world->create<RenderComponent>(se);
            src->mesh = create_sphere_mesh(1.f);
            //src->mesh = create_mesh_from_poly(box2);
            src->material = m_unit_box_material;
            auto* stc = m_world->create<TransformComponent>(se);
            float offs = 0.55f;
            stc->transform = Mat44::translate({2.f - offs, 7.f - offs, 2.f - offs});

            auto* pc2 = m_world->create<PhysicsComponent>(se);
            auto sactor = std::make_shared<DynamicActor>();

            sactor->set_shape<SphereShape>(1.f);
            //sactor->set_shape<ConvexPolyhedronShape>(box2);

            sactor->set_transform(stc->transform);

            control_actor = sactor.get();
            pc2->actor = std::move(sactor);
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

        set_background_color({0.2f, 0.2, 0.2f});

        m_world->modify_singleton<PhysicsSingleton>()->pause = true;
    }

    void update(float dt) override {

        if (gyro_actor) {
            ImGui::Begin("Gyro");
            auto& b = gyro_actor->body();
            Mat44 I = b.inv_transform() * b.local_inertia() * b.transform();
            auto Iw = I.apply_normal(b.ang_velocity());
            ImGui::Text("L (%f, %f, %f)", Iw.x, Iw.y, Iw.z);
            ImGui::Text("W %f   (%f, %f, %f)", b.ang_velocity().length(), b.ang_velocity().x, b.ang_velocity().y, b.ang_velocity().z);
            ImGui::End();
        }

        if (control_actor) {
            ImGui::Begin("Control Actor");
            auto& b = control_actor->body();
            auto m = b.transform();
            auto p = b.transform().translation();


            ImGui::DragFloat("X", &p.x, 0.01);
            ImGui::DragFloat("Y", &p.y, 0.01);
            ImGui::DragFloat("Z", &p.z, 0.01);
            ImGui::DragFloat("r_X", &control_euler.x, 0.01);
            ImGui::DragFloat("r_Y", &control_euler.y, 0.01);
            ImGui::DragFloat("r_Z", &control_euler.z, 0.01);

            m = Mat44::rotation({1.f, 0.f, 0.f}, control_euler.x)
                * Mat44::rotation({0.f, 1.f, 0.f}, control_euler.y)
                * Mat44::rotation({0.f, 0.f, 1.f}, control_euler.z);
            m.set_translation(p);
            b.set_transform(m);
            ImGui::End();
        }

        m_world->update(dt);
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

            case Key::H:
                if (is_pressed)
                    fire_capsule();
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

    void spawn_capsule(const Mat44& tr, const Vec3& velocity, const Vec3& ang_velocity, float mass) {
        auto e = m_world->create_entity();
        auto* rc = m_world->create<RenderComponent>(e);
        rc->mesh = m_capsule_mesh;
        rc->material = m_capsule_material;

        auto* tc = m_world->create<TransformComponent>(e);
        tc->transform = tr;

        auto* pc = m_world->create<PhysicsComponent>(e);

        float radius = CAPSULE_RADIUS + BLOAT;

        auto actor = std::make_shared<DynamicActor>();
        actor->set_shape<CapsuleShape>(radius, CAPSULE_HEIGHT);
        actor->set_transform(tc->transform);
        actor->body().set_velocity(velocity);
        actor->body().set_ang_velocity(ang_velocity);
        actor->body().set_mass(mass);

        float h = CAPSULE_HEIGHT + radius * 0.5f;
        float xz_inertia = mass * (3.f * radius * radius + h * h) / 12.f;
        float y_inertia = mass * radius * radius / 2.f;
        Vec3 localInertia = {xz_inertia, y_inertia, xz_inertia};
        localInertia = {1.666666f, 0.666667f, 1.666666f};
        localInertia = {1.666666f, 0.666667f, 5.666666f};
        localInertia = {1.666666f, 0.666667f, 1.666666f};
        actor->body().set_local_inertia(localInertia);
        //actor->body().set_local_inertia({mass, mass, mass});

        slope::log::info("cap mass {}", mass);
        slope::log::info("cap inertia {} {} {}", localInertia.x, localInertia.y, localInertia.z);

        actor->set_friction(0.5f);

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

    void fire_capsule() {
        auto* cam_tr = m_world->get<TransformComponent>(m_cam_entity);
        auto vel = cam_tr->transform.apply_normal({0.f, 0.f, -CAPSULE_SPEED});
        //Vec3 vel = {0.f, 0.f, -CAPSULE_SPEED};
        //Vec3 vel = {0.f, 0.f, 0.f};
        Vec3 ang_vel = cam_tr->transform.apply_normal({0.f, 8.f, 0.f});
        //Vec3 ang_vel = {0.f, 0.f, 0.f};
        spawn_capsule(cam_tr->transform, vel, ang_vel, CAPSULE_MASS);
        auto tr = Mat44::translate({0.f, 4.5f, 0.f});

        //spawn_capsule(tr, vel, ang_vel, CAPSULE_MASS);
    }

    std::shared_ptr<Mesh> m_big_box_mesh;
    std::shared_ptr<ConvexPolyhedron> m_big_box;

    std::shared_ptr<Material> m_unit_box_material;
    std::shared_ptr<Mesh> m_unit_box_mesh;
    std::shared_ptr<ConvexPolyhedron> m_unit_box;

    std::shared_ptr<Mesh> m_sphere_mesh;
    std::shared_ptr<Material> m_sphere_material;

    std::shared_ptr<Mesh> m_capsule_mesh;
    std::shared_ptr<Material> m_capsule_material;

    Narrowphase m_narrowphase;

    bool m_cam_move_mode = false;
    Entity m_cam_entity;
    std::unique_ptr<World> m_world;

    DynamicActor* gyro_actor = nullptr;

    Vec3 control_euler;
    DynamicActor* control_actor = nullptr;

    int m_mode = 2;
};

int main() {
    AppManager::run<TestApp>({1600, 1200, "Hello world"});
    return 0;
}
