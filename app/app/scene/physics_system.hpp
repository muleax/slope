#pragma once
#include "app/ecs/component.hpp"
#include "app/ecs/system.hpp"
#include "slope/dynamics/actor.hpp"
#include "slope/dynamics/dynamics_world.hpp"
#include "slope/thread/parallel_executor.hpp"
#include <memory>

namespace slope::app {

struct PhysicsSingleton : public Component<PhysicsSingleton> {
    DynamicsWorld dynamics_world;

    bool pause = false;
    bool real_time_sync = true;
    float time_factor = 1.f;

    int concurrency = 4;

    // Stats
    MovingAverage<double> cpu_time;
};

struct PhysicsComponent : public Component<PhysicsComponent> {
    BaseActor* actor = nullptr;
};

class PhysicsSystem : public System {
public:
    using System::System;
    void update(float dt) override;

private:
    void step_simulation(PhysicsSingleton* ps);

    std::unique_ptr<ParallelExecutor> m_executor;
    float m_accum_time = 0.f;
};

} // slope::app
