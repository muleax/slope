#include "app/scene/physics.hpp"
#include "app/scene/transform.hpp"

namespace slope::app {

REGISTER_COMPONENT(PhysicsComponent);
REGISTER_COMPONENT(PhysicsSingleton);

void PhysicsSystem::update(float dt) {
    auto* ps = w().get_singleton_for_write<PhysicsSingleton>();

    if (ps->pause)
        return;

    m_accum_time += dt;
    float step = 1.f / 60.f;
    if (m_accum_time < step)
        return;

    m_accum_time = fmod(m_accum_time, step);

    auto& actors = view<PhysicsComponent, TransformComponent>();

    // TODO: init views
    for (auto e : actors) {
        auto* pc = w().get_component_for_write<PhysicsComponent>(e);
        if (!pc->added) {
            ps->m_dynamics_world.add_actor(pc->actor.get());
            pc->added = true;
        }
    }

    ps->m_dynamics_world.update(step);

    for (auto e : actors) {
        auto* pc = w().get_component<PhysicsComponent>(e);
        auto* tr = w().get_component_for_write<TransformComponent>(e);
        tr->transform = pc->actor->transform();
    }
}

} // slope::app
