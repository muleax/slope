#include "app/scene/physics.hpp"
#include "app/scene/transform.hpp"
#include "app/system/utils.hpp"
#include "imgui/imgui.h"

namespace slope::app {

REGISTER_COMPONENT(PhysicsComponent);
REGISTER_COMPONENT(PhysicsSingleton);

void PhysicsSystem::step_simulation(PhysicsSingleton* ps) {
    auto& actors = view<PhysicsComponent, TransformComponent>();

    // TODO: init views
    for (auto e: actors) {
        auto* pc = w().get_component_for_write<PhysicsComponent>(e);
        if (!pc->added) {
            ps->dynamics_world.add_actor(pc->actor.get());
            pc->added = true;
        }
    }

    auto t1 = get_time();

    ps->dynamics_world.update(ps->time_step);

    ps->frame_time = (1.f - ps->frame_time_mva) * ps->frame_time + ps->frame_time_mva * (get_time() - t1);

    for (auto e: actors) {
        auto* pc = w().get_component<PhysicsComponent>(e);
        auto* tr = w().get_component_for_write<TransformComponent>(e);
        tr->transform = pc->actor->transform();
    }
}

void PhysicsSystem::process_ui_overlay(PhysicsSingleton* ps) {
    auto& world_stats = ps->dynamics_world.stats();
    auto& world_config = ps->dynamics_world.config();

    ImGui::Begin("Simulation");
    ImGui::Text("Frame Time: %.1f ms", ps->frame_time * 1000);
    ImGui::Text("Simulation Time: %.1f s", world_stats.simulation_time);
    ImGui::Text("Static Actors: %d", world_stats.static_actor_count);
    ImGui::Text("Dynamic Actors: %d", world_stats.dynamic_actor_count);
    ImGui::Text("Collisions: %d", world_stats.collision_count);
    ImGui::Text("Contacts: %d", world_stats.contact_count);

    ImGui::Separator();
    ImGui::Checkbox("Pause", &ps->pause);
    ImGui::Checkbox("Time Sync", &ps->time_sync);
    ImGui::Checkbox("Initial Randomization", &world_config.randomize_order);

    float time_step_ms = ps->time_step * 1000.f;
    ImGui::DragFloat("Time Step", &time_step_ms, 0.1f, 0.1f, 1000.f);
    ps->time_step = time_step_ms * 0.001f;

    ImGui::DragInt("Iterations", &world_config.iteration_count, 0.5f, 1, 300);
    ImGui::DragFloat("WS Normal", &world_config.warmstarting_normal, 0.002f, 0.f, 1.f);
    ImGui::DragFloat("WS Friction", &world_config.warmstarting_friction, 0.002f, 0.f, 1.f);
    ImGui::DragFloat("SOR", &world_config.sor, 0.001f, 0.f, 1.f);
    ImGui::InputFloat3("Gravity", world_config.gravity.data);

    ImGui::End();

    //static bool show_demo_window = true;
    //ImGui::ShowDemoWindow(&show_demo_window);
}

void PhysicsSystem::update(float dt) {
    auto* ps = w().get_singleton_for_write<PhysicsSingleton>();

    if (!ps->pause) {
        if (ps->time_sync) {
            m_accum_time += dt;
            if (m_accum_time >= ps->time_step) {
                m_accum_time = fmod(m_accum_time, ps->time_step);
                step_simulation(ps);
            }
        } else {
            step_simulation(ps);
        }
    }

    process_ui_overlay(ps);
}

} // slope::app
