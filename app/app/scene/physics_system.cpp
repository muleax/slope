#include "app/scene/physics_system.hpp"
#include "app/scene/transform.hpp"
#include "app/system/utils.hpp"

namespace slope::app {

REGISTER_COMPONENT(PhysicsComponent);
REGISTER_COMPONENT(PhysicsSingleton);

void PhysicsSystem::step_simulation(PhysicsSingleton* ps) {
    auto& actors = view<PhysicsComponent, TransformComponent>();

    // TODO: init views
    for (auto e: actors) {
        auto* pc = w().modify<PhysicsComponent>(e);
        if (!pc->added) {
            ps->dynamics_world.add_actor(pc->actor.get());
            pc->added = true;
        }
    }

    auto t1 = get_time();

    ps->dynamics_world.update(ps->time_step);

    ps->cpu_time.update(get_time() - t1);

    for (auto e: actors) {
        auto* pc = w().get<PhysicsComponent>(e);
        auto* tr = w().modify<TransformComponent>(e);
        tr->transform = pc->actor->transform();
    }
}

void PhysicsSystem::update(float dt) {
    auto* ps = w().modify_singleton<PhysicsSingleton>();

    if (!ps->pause) {
        if (ps->real_time_sync) {
            m_accum_time += dt * ps->time_factor;
            if (m_accum_time >= ps->time_step) {
                m_accum_time = fmod(m_accum_time, ps->time_step);
                step_simulation(ps);
            }
        } else {
            step_simulation(ps);
        }
    }
}

} // slope::app
