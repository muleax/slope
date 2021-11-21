#pragma once
#include "app/ecs/component.hpp"
#include "app/ecs/system.hpp"
#include "slope/dynamics/actor.hpp"
#include "slope/dynamics/dynamics_world.hpp"
#include <memory>

namespace slope::app {

struct PhysicsSingleton : public Component<PhysicsSingleton> {
    bool pause = false;
    float time_step = 1.f / 60.f;
    DynamicsWorld m_dynamics_world;

    // Stats
    double frame_time = 0.0;
    double frame_time_mva = 0.1;
    float simulation_time = 0.f;
};

struct PhysicsComponent : public Component<PhysicsComponent> {
    bool added = false;
    std::shared_ptr<BaseActor> actor;
};

class PhysicsSystem : public System {
public:
    using System::System;
    void update(float dt) override;

private:
    void step_simulation(PhysicsSingleton* ps);
    void draw_stats(PhysicsSingleton* ps);

    float m_accum_time = 0.f;
};

} // slope::app
