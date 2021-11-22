#pragma once
#include "app/ecs/component.hpp"
#include "app/ecs/system.hpp"
#include "slope/dynamics/actor.hpp"
#include "slope/dynamics/dynamics_world.hpp"
#include <memory>

namespace slope::app {

struct PhysicsSingleton : public Component<PhysicsSingleton> {
    DynamicsWorld dynamics_world;

    bool pause = false;
    bool time_sync = true;
    float time_step = 1.f / 60.f;

    // Stats
    double frame_time = 0.0;
    double frame_time_mva = 0.1;
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
    void process_ui_overlay(PhysicsSingleton* ps);

    float m_accum_time = 0.f;
};

} // slope::app
