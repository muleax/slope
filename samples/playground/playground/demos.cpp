#include "playground/demos.hpp"
#include "imgui/imgui.h"

void Demo::create_floor()
{
    float floor_size = 500.f;
    auto floor_geom = ConvexPolyhedronFactory().box(Vec3{floor_size, 1.f, floor_size}, Vec3{0.f, 0.f, 0.f});
    auto ground_mesh = MeshFactory().from_polyhedron(floor_geom);

    auto e = w()->create_entity();
    auto* rc = w()->create<RenderComponent>(e);
    rc->mesh = ground_mesh;

    rc->material = std::make_shared<Material>(DefaultShaders::mesh_shader());
    rc->material->set_ambient_strength(0.1f);
    rc->material->set_color({0.8, 0.8, 0.9});

    auto* tc = w()->create<TransformComponent>(e);
    tc->transform = Mat44::translate({0.f, -1.f, 0.f});

    auto* pc = w()->create<PhysicsComponent>(e);
    pc->actor = std::make_shared<StaticActor>();
    pc->actor->set_shape<BoxShape>(Vec3{floor_size, 1.f, floor_size});
    pc->actor->set_friction(0.5f);
    pc->actor->set_transform(tc->transform);
}

void StackDemo::reset()
{
    create_floor();

    float skew = 0.1f;
    for (int j = 0; j < m_height; j++) {
        Mat44 tr = Mat44::rotation({0.f, 1.f, 0.f}, j * PI * 0.f);
        tr.set_translation({skew * j, 0.5f + j * 2.02f, skew * j});
        m_spawner->spawn_sphere(tr, {}, 1.f, 1.f);
    }
}

void TriangleStackDemo::reset()
{
    create_floor();

    auto& config = physics_config();
    config.randomize_order = true;
    config.enable_velocity_dependent_friction = true;
    config.solver_config.use_simd = true;
    config.solver_config.iteration_count = 30;

    //auto rot = Mat44::rotation({0.f, 1.f, 0.f}, 0.5f);
    float spacing = 0.f;
    for (int j = 0; j < m_height; j++) {
        for (int i = 0; i < m_height - j; i++) {
            for (int k = 0; k < 1; k++) {
                Mat44 tr = Mat44::rotation({0.f, 1.f, 0.f}, j * PI * 0.f);
                tr.set_translation(
                    {-float(m_height) / 2 + i * (1.f + spacing) + j * (0.5f + spacing / 2), -0.001f + j * 0.999f,
                     (j % 2) * spacing * 0.f + k * 1.0f});
                //tr *= rot;
                m_spawner->spawn_box(tr, {}, 1.f, Vec3{1.f});
            }
        }
    }
}

void StressTestDemo::reset()
{
    create_floor();

    auto& config = physics_config();
    //physics_single->dynamics_world.config().enable_constraint_resolving = false;
    //physics_single->dynamics_world.config().enable_integration = true;
    config.randomize_order = true;
    config.enable_velocity_dependent_friction = true;
    config.solver_config.iteration_count = 5;
    config.solver_config.use_simd = true;


    float spacing = 1.5f;
    for (int i = 0; i < m_width; i++) {
        for (int j = 0; j < m_height; j++) {
            for (int k = 0; k < m_width; k++) {
                Mat44 tr = Mat44::rotation({0.f, 1.f, 0.f}, j * PI * 0.f);
                tr.set_translation(
                    {(float) i * spacing + j * 0.2f, (float) j * spacing + 0.5f, (float) k * spacing + j * 0.2f});

                m_spawner->spawn_box(tr, {}, 1.f, Vec3{1.f});
            }
        }
    }
}

void CollisionDemo::reset()
{
    auto& config = physics_config();
    config.gravity.set_zero();
    config.enable_constraint_resolving = false;
    config.delay_integration = true;

    MeshFactory mesh_factory;
    ConvexPolyhedronFactory poly_factory;
    auto box = poly_factory.box(Vec3{1.f, 1.f, 1.f}, Vec3{0.f, 0.f, 0.f});

    auto material = std::make_shared<Material>(DefaultShaders::mesh_shader());
    material->set_ambient_strength(0.2f);
    material->set_color({0.9, 0.75, 0.0});

    auto e = w()->create_entity();
    auto* rc = w()->create<RenderComponent>(e);
    rc->mesh = mesh_factory.from_polyhedron(box);
    rc->material = material;
    auto* tc = w()->create<TransformComponent>(e);
    tc->transform = Mat44::translate({0.f, 5.f, 0.f});

    auto* pc = w()->create<PhysicsComponent>(e);
    auto actor = std::make_shared<DynamicActor>();
    //actor->set_shape<ConvexPolyhedronShape>(box);
    actor->set_shape<BoxShape>(Vec3{1.f, 1.f, 1.f});
    actor->set_transform(tc->transform);
    pc->actor = std::move(actor);

    auto box2 = poly_factory.box(Vec3{1.f, 1.f, 1.f}, Vec3{0.f, 0.f, 0.f});

    auto se = w()->create_entity();
    auto* src = w()->create<RenderComponent>(se);
    src->mesh = mesh_factory.from_sphere(1.f);
    //src->mesh = mesh_factory.from_polyhedron(box2);
    src->material = material;
    auto* stc = w()->create<TransformComponent>(se);
    float offs = 0.f;
    stc->transform = Mat44::translate({0.98f - offs, 5.f - offs, 0.f - offs});

    auto* pc2 = w()->create<PhysicsComponent>(se);
    auto control_actor = std::make_shared<DynamicActor>();

    control_actor->set_shape<SphereShape>(1.f);

    control_actor->set_transform(stc->transform);

    m_control_actor = control_actor.get();
    pc2->actor = std::move(control_actor);
}

void CollisionDemo::update(float dt)
{
    if (m_control_actor) {
        ImGui::Begin("Control Actor");
        auto& b = m_control_actor->body();
        auto m = b.transform();
        auto p = b.transform().translation();


        ImGui::DragFloat("X", &p.x, 0.01);
        ImGui::DragFloat("Y", &p.y, 0.01);
        ImGui::DragFloat("Z", &p.z, 0.01);
        ImGui::DragFloat("Pitch", &m_control_euler.x, 0.01);
        ImGui::DragFloat("Yaw", &m_control_euler.y, 0.01);
        ImGui::DragFloat("Roll", &m_control_euler.z, 0.01);

        m = Mat44::rotation({1.f, 0.f, 0.f}, m_control_euler.x)
            * Mat44::rotation({0.f, 1.f, 0.f}, m_control_euler.y)
            * Mat44::rotation({0.f, 0.f, 1.f}, m_control_euler.z);
        m.set_translation(p);
        b.set_transform(m);
        ImGui::End();
    }
}

void TennisRacketDemo::reset()
{
    auto& config = physics_config();
    config.gravity.set_zero();

    auto actor = m_spawner->spawn_box(Mat44::translate({0.f, 5.f, 0.f}), {}, 1.f, Vec3{2.f, 4.f, 0.4f});
    actor->body().set_ang_velocity({8.f, 5.85f, 1.f});

    Vec3 localInertia = {1.666666f, 0.666667f, 5.666666f};
    actor->body().set_local_inertia(localInertia);
}
