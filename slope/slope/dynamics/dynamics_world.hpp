#pragma once
#include "slope/dynamics/constraint_solver.hpp"
#include "slope/dynamics/actor.hpp"
#include "slope/containers/vector.hpp"
#include "slope/containers/unordered_map.hpp"
#include "slope/collision/contact_manifold.hpp"
#include "slope/collision/convex_polyhedron_collider.hpp"

// TODO: do not use pointers for hashing
namespace slope {
using ManifoldCacheKey = std::pair<const void*, const void*>;
}

namespace std
{
template<> struct hash<slope::ManifoldCacheKey>
{
    std::size_t operator()(const slope::ManifoldCacheKey& key) const noexcept
    {
        return reinterpret_cast<size_t>(key.first) ^ reinterpret_cast<size_t>(key.second);
    }
};
}

namespace slope {

class DynamicsWorld {
public:
    void add_actor(BaseActor* actor);
    void remove_actor(BaseActor* actor);

    void update(float dt);

    void set_gravity(const Vec3& value) { m_gravity = value; }
    const Vec3& gravity() const { return m_gravity; }

    void set_warstarting_normal(float value) { m_warstarting_normal = value; }
    float warstarting_normal() const { return m_warstarting_normal; }

    void set_warstarting_friction(float value) { m_warstarting_friction = value; }
    float warstarting_friction() const { return m_warstarting_friction; }

    void randomize_order(bool value) { m_randomize_order = value; }
    bool randomize_order() const { return m_randomize_order; }

    ConstraintSolver& solver() { return m_solver; }
    const ConstraintSolver& solver() const { return m_solver; }

    uint32_t frame_id() const { return m_frame_id; }
    float simulation_time() const { return m_simulation_time; }

private:
    struct ManifoldCache {
        ContactManifold manifold;
        BaseActor* actor1 = nullptr;
        BaseActor* actor2 = nullptr;
    };

    struct PendingContact {
        const ManifoldCache* mf_cache;
        ManifoldPoint* mf_point;
    };

    template<class T>
    void remove_actor_impl(Vector<T>& container, BaseActor* actor);

    void apply_gravity();
    void perform_collision_detection();
    void collide(BaseActor& actor1, BaseActor& actor2);
    void apply_contacts();
    void cache_lambdas();
    void integrate_bodies();
    void refresh_manifolds();

    ConstraintSolver m_solver;

    Vector<DynamicActor*> m_dynamic_actors;
    Vector<StaticActor*> m_static_actors;

    Vector<ContactGeom> m_geom_buffer;
    Vector<PendingContact> m_pending_contacts;
    ConvexPolyhedronCollider m_collider;

    UnorderedMap<ManifoldCacheKey, ManifoldCache> m_manifolds;

    bool m_randomize_order = true;
    float m_warstarting_normal = 0.83f;
    float m_warstarting_friction = 0.83f;
    Vec3 m_gravity = {0.f, -9.81f, 0.f};

    uint32_t m_frame_id = 0;
    float m_simulation_time = 0.f;
};

} // slope
