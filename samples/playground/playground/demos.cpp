#include "playground/demos.hpp"
#include "imgui/imgui.h"
#include <random>

void Demo::create_floor()
{
    float floor_size = 500.f;
    auto floor_geom = ConvexPolyhedronFactory().box(vec3{floor_size, 1.f, floor_size}, vec3{0.f, 0.f, 0.f});
    auto ground_mesh = MeshFactory().from_polyhedron(floor_geom);

    auto e = w()->create_entity();
    auto* rc = w()->create<RenderComponent>(e);
    rc->mesh = ground_mesh;

    rc->material = std::make_shared<Material>(DefaultShaders::mesh_shader());
    rc->material->set_ambient_strength(0.1f);
    rc->material->set_color({0.8, 0.8, 0.9});

    auto* tc = w()->create<TransformComponent>(e);
    tc->transform = mat44::translate({0.f, -0.5f, 0.f});

    auto* pc = w()->create<PhysicsComponent>(e);
    pc->actor = dynamics_world()->create_kinematic_actor();
    dynamics_world()->assign_shape(pc->actor, BoxShape({floor_size, 1.f, floor_size}));

    pc->actor->set_friction(0.5f);
    pc->actor->set_transform(tc->transform);
}

void TriangleStackDemo::apply_default_config()
{
    auto config = dynamics_world()->config();
    config.randomize_order = true;
    config.solver_config.iteration_count = 30;
    dynamics_world()->update_config(config);
}

void TriangleStackDemo::init()
{
    create_floor();

    int height = 30;

    float spacing = 0.f;
    for (int j = 0; j < height; j++) {
        for (int i = 0; i < height - j; i++) {
            for (int k = 0; k < 1; k++) {
                auto tr = mat44::translate({
                    static_cast<float>(-height / 2 + i * (1.f + spacing) + j * (0.5f + spacing / 2)),
                    static_cast<float>(j * 0.999f + 0.49f),
                    static_cast<float>((j % 2) * spacing * 0.f + k * 1.0f)
                });
                m_spawner->spawn_box(tr, {}, 1.f, vec3{1.f});
            }
        }
    }
}

void StackDemo::apply_default_config()
{
    auto config = dynamics_world()->config();
    config.randomize_order = true;
    config.solver_config.iteration_count = 30;
    dynamics_world()->update_config(config);
}

void StackDemo::init()
{
    create_floor();

    for (int i = 0; i < 20; i++) {
        m_spawner->spawn_capsule(mat44::translate({20.f, 2.f + i * 3.f, 0.f}), vec3::zero(), vec3::zero(), 1.f, 0.5f, 1.f);
    }

    for (int i = 0; i < 20; i++) {
        m_spawner->spawn_sphere(mat44::translate({10.f, 1.f + i * 3.f, 0.f}), vec3::zero(), 2.f, 1.f);
    }

    for (int i = 0; i < 10; i++) {
        m_spawner->spawn_box(mat44::translate({0.f, 0.5f + i * 1.5f, 0.f}), vec3::zero(), 1.5f, {1.f, 1.f, 1.f});
    }

    BodySpawner::PolyhedronDimensions dims;
    dims.corners = 8;
    dims.h = 1.f;
    dims.w_lo = 1.f;
    dims.w_hi = 1.7f;
    for (int i = 0; i < 5; i++) {
        float h = 1.f;
        m_spawner->spawn_polyhedron(mat44::translate({-10.f, 0.8f * h + i * h * 1.5f, 0.f}), vec3::zero(), vec3::zero(), 2.f, dims);
    }

    dims.corners = 16;
    for (int i = 0; i < 5; i++) {
        float h = 1.f;
        m_spawner->spawn_polyhedron(mat44::translate({-20.f, 0.8f * h + i * h * 1.5f, 0.f}), vec3::zero(), vec3::zero(), 2.f, dims);
    }

    dims.corners = 24;
    for (int i = 0; i < 5; i++) {
        float h = 1.f;
        m_spawner->spawn_polyhedron(mat44::translate({-30.f, 0.8f * h + i * h * 1.5f, 0.f}), vec3::zero(), vec3::zero(), 2.f, dims);
    }
}

void Stress1KDemo::init()
{
    auto config = dynamics_world()->config();
    config.randomize_order = true;
    config.solver_config.iteration_count = 5;
    dynamics_world()->update_config(config);
}

void Stress1KDemo::apply_default_config()
{
    create_floor();

    int height = 10;
    int width = 10;
    int depth = 10;

    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            for (int k = 0; k < depth; k++) {
                auto tr = mat44::translate({
                    static_cast<float>((i - width / 2) * 1.f + 0.5f * (j % 2)),
                    static_cast<float>(j * 1.f + 0.49f),
                    static_cast<float>(k * 1.f + 0.5f * (i % 2) - 10.f)
                });

                m_spawner->spawn_box(tr, vec3::zero(), 1.f, vec3{1.f});
            }
        }
    }
}

void Stress6KDemo::init()
{
    auto config = dynamics_world()->config();
    config.randomize_order = true;
    config.solver_config.iteration_count = 5;
    dynamics_world()->update_config(config);
}

void Stress6KDemo::apply_default_config()
{
    create_floor();

    int height = 60;
    int width = 10;
    int depth = 10;

    float skew = 0.2f;
    float spacing = 1.2f;
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            for (int k = 0; k < depth; k++) {
                auto tr = mat44::translate({
                    static_cast<float>(i * spacing + j * skew - 20.f),
                    static_cast<float>(j * spacing + 0.5f),
                    static_cast<float>(k * spacing + j * skew - 50.f)
                });

                m_spawner->spawn_box(tr, vec3::zero(), 1.f, vec3{1.f});
            }
        }
    }
}

void Stress10KDemo::apply_default_config()
{
    auto config = dynamics_world()->config();
    config.randomize_order = true;
    config.solver_config.iteration_count = 5;
    dynamics_world()->update_config(config);
}

void Stress10KDemo::init()
{
    create_floor();

    int height = 5;
    int width = 40;
    int depth = 50;

    float y_spacing = 1.f;
    float xz_spacing = 3.f;
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            for (int k = 0; k < depth; k++) {
                auto tr = mat44::translate({
                    static_cast<float>((i - width / 2) * xz_spacing),
                    static_cast<float>(j * y_spacing + 0.5f),
                    static_cast<float>((k - depth / 2) * xz_spacing)
                });

                m_spawner->spawn_box(tr, vec3::zero(), 1.f, vec3{1.f});
            }
        }
    }
}

void SphericalJointDemo::apply_default_config()
{
    auto config = dynamics_world()->config();
    config.solver_config.iteration_count = 30;
    dynamics_world()->update_config(config);
}

void SphericalJointDemo::init()
{
    static constexpr int CHAIN_LENGTH = 12;

    create_floor();

    float y = 14.f;
    float x = 0.f;

    float offset = 0.9f;

    DynamicActor* prev_actor = nullptr;
    for (int i = 0; i < CHAIN_LENGTH; i++) {
        auto tr = mat44::rotation({0.f, 0.f, 1.f}, -slope::PI * 0.5f) * mat44::translate({x, y, 0.f});
        auto* new_actor = m_spawner->spawn_capsule(tr, vec3::zero(), vec3::zero(), 0.5f, 0.25f, 0.8f);

        auto* joint = dynamics_world()->create_joint<SphericalJoint>(new_actor, prev_actor);

        joint->set_damping(0.5f);
        joint->set_anchor1({ 0.f, offset, 0.f });
        joint->set_anchor2(prev_actor ? vec3{0.f, -offset, 0.f} : vec3{x + offset, y, 0.f});

        x -= 2.f * offset;
        prev_actor = new_actor;
    }
}

void TennisRacketDemo::init()
{
    std::uniform_real_distribution<float> dist(-8.f, 8.f);

    auto config = dynamics_world()->config();
    config.gravity.set_zero();
    dynamics_world()->update_config(config);

    auto actor = m_spawner->spawn_box(mat44::translate({0.f, 5.f, 0.f}), {}, 1.f, vec3{2.f, 4.f, 0.4f});
    actor->body().set_ang_velocity({dist(m_mt_engine), dist(m_mt_engine), dist(m_mt_engine)});
}

void TennisRacketDemo::fini()
{
    auto config = dynamics_world()->config();
    config.gravity = {0.f, -9.81f, 0.f};
    dynamics_world()->update_config(config);
}

void ContactGenerationDemo::init()
{
    auto config = dynamics_world()->config();
    config.gravity.set_zero();
    config.enable_constraint_resolving = false;
    config.delay_integration = true;
    dynamics_world()->update_config(config);

    auto material = std::make_shared<Material>(DefaultShaders::mesh_shader());
    material->set_ambient_strength(0.2f);
    material->set_color({0.9, 0.75, 0.0});

    MeshFactory mesh_factory;
    ConvexPolyhedronFactory poly_factory;

    {
        vec3 static_box_size = {1.5f, 1.5f, 1.5f};
        auto static_box = poly_factory.box(static_box_size, vec3{0.f, 0.f, 0.f});

        auto e = w()->create_entity();
        auto* rc = w()->create<RenderComponent>(e);
        rc->mesh = mesh_factory.from_polyhedron(static_box);
        rc->material = material;
        auto* tc = w()->create<TransformComponent>(e);
        tc->transform = mat44::translate({0.f, 5.f, 0.f});

        auto* pc = w()->create<PhysicsComponent>(e);
        pc->actor = dynamics_world()->create_dynamic_actor();
        dynamics_world()->assign_shape(pc->actor, BoxShape(static_box_size));

        pc->actor->set_transform(tc->transform);
    }

    {
        float static_sphere_radius = 1.2f;

        auto e = w()->create_entity();
        auto* rc = w()->create<RenderComponent>(e);
        rc->mesh = mesh_factory.from_sphere(static_sphere_radius);
        rc->material = material;
        auto* tc = w()->create<TransformComponent>(e);
        tc->transform = mat44::translate({5.f, 5.f, 0.f});

        auto* pc = w()->create<PhysicsComponent>(e);
        pc->actor = dynamics_world()->create_dynamic_actor();
        dynamics_world()->assign_shape(pc->actor, SphereShape(static_sphere_radius));

        pc->actor->set_transform(tc->transform);
    }

    {
        vec3 dynamic_box_size = {1.f, 1.f, 1.f};
        auto dynamic_box = poly_factory.box(dynamic_box_size, vec3{0.f, 0.f, 0.f});

        auto se = w()->create_entity();
        auto* src = w()->create<RenderComponent>(se);
        //src->mesh = mesh_factory.from_sphere(1.f);
        src->mesh = mesh_factory.from_polyhedron(dynamic_box);
        src->material = material;
        auto* stc = w()->create<TransformComponent>(se);
        float offs = 0.f;
        stc->transform = mat44::translate({0.98f - offs, 5.f - offs, 0.f - offs});

        m_controlled_actor = dynamics_world()->create_dynamic_actor();
        dynamics_world()->assign_shape(m_controlled_actor, BoxShape(dynamic_box_size));

        m_controlled_actor->set_transform(stc->transform);
        auto* pc = w()->create<PhysicsComponent>(se);
        pc->actor = m_controlled_actor;
    }
}

void ContactGenerationDemo::fini()
{
    auto config = dynamics_world()->config();
    config.gravity = {0.f, -9.81f, 0.f};
    config.enable_constraint_resolving = true;
    config.delay_integration = true;
    dynamics_world()->update_config(config);
}

void ContactGenerationDemo::update(float dt)
{
    if (m_controlled_actor) {

        ImGui::SetNextWindowPos(ImVec2(600, 15), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(230, 190), ImGuiCond_FirstUseEver);
        ImGui::Begin("Controlled Actor");
        auto& b = m_controlled_actor->body();
        auto m = b.transform();
        auto p = b.transform().translation();


        ImGui::DragFloat("X", &p.x, 0.01);
        ImGui::DragFloat("Y", &p.y, 0.01);
        ImGui::DragFloat("Z", &p.z, 0.01);
        ImGui::DragFloat("Pitch", &m_control_euler.x, 0.01);
        ImGui::DragFloat("Yaw", &m_control_euler.y, 0.01);
        ImGui::DragFloat("Roll", &m_control_euler.z, 0.01);

        m = mat44::rotation({1.f, 0.f, 0.f}, m_control_euler.x)
            * mat44::rotation({0.f, 1.f, 0.f}, m_control_euler.y)
            * mat44::rotation({0.f, 0.f, 1.f}, m_control_euler.z);
        m.set_translation(p);
        b.set_transform(m);
        ImGui::End();
    }
}
