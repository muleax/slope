#include "app/scene/physics_system.hpp"
#include "app/scene/transform.hpp"
#include "app/system/utils.hpp"
#include "slope/core/config.hpp"

namespace slope::app {

REGISTER_COMPONENT(PhysicsComponent);
REGISTER_COMPONENT(PhysicsSingleton);

void PhysicsSystem::step_simulation(PhysicsSingleton* ps) {
    if (!m_executor || m_executor->concurrency() != ps->concurrency) {
        m_executor = std::make_unique<ParallelExecutor>(ps->concurrency);
        ps->dynamics_world.setup_executor(*m_executor);
    }

    auto t1 = get_time();

    {
        SL_FRAME_MARK_START("Physics")

        m_executor->run().wait();

        SL_FRAME_MARK_END("Physics")
    }

    ps->cpu_time.update(get_time() - t1);

    for (auto e : view<PhysicsComponent, TransformComponent>()) {
        auto* pc = w().get<PhysicsComponent>(e);
        auto* tr = w().modify<TransformComponent>(e);
        tr->transform = pc->actor->transform();
    }
}

void PhysicsSystem::update(float dt) {
    auto* ps = w().modify_singleton<PhysicsSingleton>();

    float fixed_dt = ps->dynamics_world.config().solver_config.time_interval;

    if (!ps->pause) {
        if (ps->real_time_sync) {
            m_accum_time += dt * ps->time_factor;
            if (m_accum_time >= fixed_dt) {
                m_accum_time = fmod(m_accum_time, fixed_dt);
                step_simulation(ps);
            }
        } else {
            step_simulation(ps);
        }
    }
}

} // slope::app
