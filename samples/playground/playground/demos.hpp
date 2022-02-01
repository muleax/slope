#pragma once

#include "playground/helpers.hpp"

class Demo {
public:
    explicit Demo(BodySpawner* spawner) : m_spawner(spawner) {}
    virtual ~Demo() = default;

    virtual void        reset() { create_floor(); }
    virtual void        update(float dt) {}
    virtual const char* name() = 0;

protected:
    World*              w() { return m_spawner->world(); }
    const World*        w() const { return m_spawner->world(); }
    auto&               physics_config() { return w()->modify_singleton<PhysicsSingleton>()->dynamics_world.config(); }

    void                create_floor();

    BodySpawner* m_spawner = nullptr;
};

class StackDemo : public Demo {
public:
    using Demo::Demo;

    void        reset() override;
    const char* name() override { return "Stack"; }

private:
    int m_height = 5;
};

class TriangleStackDemo : public Demo {
public:
    using Demo::Demo;

    void        reset() override;
    const char* name() override { return "Triangle Stack"; }

private:
    int m_height = 30;
};

class StressTestDemo : public Demo {
public:
    using Demo::Demo;

    void        reset() override;
    const char* name() override { return "Stress Test"; }

private:
    int m_height = 47;
    int m_width = 8;
};

class CollisionDemo : public Demo {
public:
    using Demo::Demo;

    void        reset() override;
    void        update(float dt) override;
    const char* name() override { return "Contact Generation"; }

private:
    Vec3 m_control_euler;
    DynamicActor* m_control_actor = nullptr;
};

class TennisRacketDemo : public Demo {
public:
    using Demo::Demo;

    void        reset() override;
    const char* name() override { return "Tennis Racket"; }

private:
    int m_height = 47;
    int m_width = 8;
};
