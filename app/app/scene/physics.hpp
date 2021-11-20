#pragma once
#include "app/ecs/component.hpp"
#include "app/ecs/system.hpp"
#include "slope/dynamics/actor.hpp"
#include "slope/dynamics/dynamics_world.hpp"
#include <memory>

namespace slope::app {

struct PhysicsSingleton : public Component<PhysicsSingleton> {
    bool pause = false;
    DynamicsWorld m_dynamics_world;
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
    float m_accum_time = 0.f;
};

} // slope::app
