#pragma once

#include "app/render/render_system.hpp"
#include "app/render/mesh.hpp"
#include "app/render/material.hpp"
#include "app/render/default_shaders.hpp"
#include "app/scene/transform.hpp"
#include "app/scene/physics_system.hpp"
#include "app/ecs/world.hpp"
#include "slope/collision/geometry.hpp"
#include "slope/collision/shape/sphere_shape.hpp"
#include "slope/collision/shape/box_shape.hpp"
#include "slope/collision/shape/capsule_shape.hpp"
#include "slope/containers/vector.hpp"
#include <memory>

using namespace slope;
using namespace slope::app;

static constexpr float COLLISION_BLOAT = 0.01f;

class MeshFactory {
public:
    std::shared_ptr<Mesh> from_polyhedron(const std::shared_ptr<ConvexPolyhedron>& geom);
    std::shared_ptr<Mesh> from_sphere(float radius);
    std::shared_ptr<Mesh> from_capsule(float radius, float axis_length);

private:
    TrimeshFactory m_trimesh_factory;
};

class BodySpawner {
public:
    explicit BodySpawner(World* world)
    : m_world(world)
    , m_dynamics_world(&m_world->modify_singleton<PhysicsSingleton>()->dynamics_world) {}

    DynamicActor*   spawn_box(const Mat44& tr, const Vec3& velocity, float mass, const Vec3& size);
    DynamicActor*   spawn_sphere(const Mat44& tr, const Vec3& velocity, float mass, float radius);
    DynamicActor*   spawn_capsule(const Mat44& tr, const Vec3& velocity, const Vec3& ang_velocity, float mass, float radius, float height);

    World*          world() { return m_world; }
    DynamicsWorld*  dynamics_world() { return m_dynamics_world; }

private:
    template<class T>
    struct MeshData {
        T dimensions;
        std::shared_ptr<Mesh> mesh;
    };

    template<class T, class F>
    std::shared_ptr<Mesh> get_mesh(Vector<MeshData<T>>& container, T dimensions, F&& ctor);

    Vector<MeshData<Vec3>> m_box_meshes;
    std::shared_ptr<Material> m_box_material;

    Vector<MeshData<float>> m_sphere_meshes;
    std::shared_ptr<Material> m_sphere_material;

    Vector<MeshData<Vec2>> m_capsule_meshes;
    std::shared_ptr<Material> m_capsule_material;

    MeshFactory m_mesh_factory;
    ConvexPolyhedronFactory m_poly_factory;

    World* m_world = nullptr;
    DynamicsWorld* m_dynamics_world = nullptr;
};

template<class T, class F>
std::shared_ptr<Mesh> BodySpawner::get_mesh(Vector<BodySpawner::MeshData<T>>& container, T dimensions, F&& ctor)
{
    std::shared_ptr<Mesh> result;

    auto it = std::find_if(container.begin(), container.end(),
                           [&dimensions](auto& e) { return e.dimensions == dimensions; });
    if (it == container.end()) {
        result = ctor();
        container.push_back({dimensions, result});
    } else {
        result = it->mesh;
    }

    return result;
}
