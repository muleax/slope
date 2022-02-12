#pragma once
#include "playground/helpers.hpp"
#include "slope/dynamics/joint/spherical_joint.hpp"
#include <optional>
#include <random>

class Demo {
public:
    explicit Demo(BodySpawner* spawner) : m_spawner(spawner) {}
    virtual ~Demo() = default;

    virtual void        init() { create_floor(); }
    virtual void        apply_default_config() {}
    virtual void        fini() {}
    virtual void        update(float dt) {}
    virtual const char* name() = 0;

protected:
    World*              w() { return m_spawner->world(); }
    DynamicsWorld*      dynamics_world() { return m_spawner->dynamics_world(); }

    void                create_floor();

    BodySpawner* m_spawner = nullptr;
};

class TriangleStackDemo : public Demo {
public:
    using Demo::Demo;

    void        init() override;
    void        apply_default_config() override;
    const char* name() override { return "Triangle Stack"; }
};

class StackDemo : public Demo {
public:
    using Demo::Demo;

    void        init() override;
    void        apply_default_config() override;
    const char* name() override { return "Stack"; }
};


class Stress1KDemo : public Demo {
public:
    using Demo::Demo;

    void        init() override;
    void        apply_default_config() override;
    const char* name() override { return "1K Boxes"; }
};

class Stress6KDemo : public Demo {
public:
    using Demo::Demo;

    void        init() override;
    void        apply_default_config() override;
    const char* name() override { return "6K Boxes"; }
};

class Stress10KDemo : public Demo {
public:
    using Demo::Demo;

    void        init() override;
    void        apply_default_config() override;
    const char* name() override { return "10K Boxes"; }
};

class SphericalJointDemo : public Demo {
public:
    using Demo::Demo;

    void        init() override;
    void        apply_default_config() override;
    const char* name() override { return "Spherical Joint"; }

private:
};

class TennisRacketDemo : public Demo {
public:
    using Demo::Demo;

    void        init() override;
    void        fini() override;
    const char* name() override { return "Tennis Racket"; }

private:
    int m_height = 47;
    int m_width = 8;

    std::mt19937 m_mt_engine;
};

class ContactGenerationDemo : public Demo {
public:
    using Demo::Demo;

    void        init() override;
    void        fini() override;
    void        update(float dt) override;
    const char* name() override { return "Contact Generation"; }

private:
    vec3 m_control_euler;
    DynamicActor* m_controlled_actor = nullptr;
};
