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
            ps->m_dynamics_world.add_actor(pc->actor.get());
            pc->added = true;
        }
    }

    auto t1 = get_time();

    ps->m_dynamics_world.update(ps->time_step);

    ps->frame_time = (1.f - ps->frame_time_mva) * ps->frame_time + ps->frame_time_mva * (get_time() - t1);
    ps->simulation_time += ps->time_step;

    for (auto e: actors) {
        auto* pc = w().get_component<PhysicsComponent>(e);
        auto* tr = w().get_component_for_write<TransformComponent>(e);
        tr->transform = pc->actor->transform();
    }
}

void PhysicsSystem::draw_stats(PhysicsSingleton* ps) {
    ImGui::Begin("Stats");
    ImGui::Text("frame time: %.1f ms", ps->frame_time * 1000);
    ImGui::Text("simulation time: %.1f s", ps->simulation_time);
    ImGui::End();
}

void PhysicsSystem::update(float dt) {
    auto* ps = w().get_singleton_for_write<PhysicsSingleton>();

    if (!ps->pause) {
        m_accum_time += dt;
        if (m_accum_time >= ps->time_step) {
            m_accum_time = fmod(m_accum_time, ps->time_step);

            step_simulation(ps);
        }
    }

    draw_stats(ps);
}

} // slope::app
