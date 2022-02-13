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
#include "slope/core/vector.hpp"
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
    struct PolyhedronDimensions {
        int corners = 8;
        float h = 1.f;
        float w_lo = 1.f;
        float w_hi = 1.7f;

        bool operator==(const PolyhedronDimensions& other) const {
            return corners == other.corners && h == other.h && w_lo == other.w_lo && w_hi == other.w_hi; }
    };

    explicit BodySpawner(World* world)
    : m_world(world)
    , m_dynamics_world(&m_world->modify_singleton<PhysicsSingleton>()->dynamics_world) {}

    DynamicActor*   spawn_box(const mat44& tr, const vec3& velocity, float mass, const vec3& size);
    DynamicActor*   spawn_sphere(const mat44& tr, const vec3& velocity, float mass, float radius);
    DynamicActor*   spawn_capsule(const mat44& tr, const vec3& velocity, const vec3& ang_velocity, float mass, float radius, float height);
    DynamicActor*   spawn_polyhedron(const mat44& tr, const vec3& velocity, const vec3& ang_velocity, float mass, PolyhedronDimensions dimensions);

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

    std::shared_ptr<ConvexPolyhedron> create_polyhedron_geom(PolyhedronDimensions dimensions);

    Vector<MeshData<vec3>> m_box_meshes;
    std::shared_ptr<Material> m_box_material;

    Vector<MeshData<float>> m_sphere_meshes;
    std::shared_ptr<Material> m_sphere_material;

    Vector<MeshData<vec2>> m_capsule_meshes;
    std::shared_ptr<Material> m_capsule_material;

    Vector<MeshData<PolyhedronDimensions>> m_phd_meshes;
    std::shared_ptr<Material> m_phd_material;
    Vector<uint32_t> m_face_indices;

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
