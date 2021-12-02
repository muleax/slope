#pragma once
#include "slope/dynamics/constraint_solver.hpp"
#include "slope/dynamics/actor.hpp"
#include "slope/containers/vector.hpp"
#include "slope/containers/unordered_map.hpp"
#include "slope/collision/contact_manifold.hpp"
#include "slope/collision/narrowphase/narrowphase.hpp"
#include "slope/debug/debug_drawer.hpp"
#include <memory>

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
    enum class NpBackendHint {
        GJK_EPA,
        SAT
    };

    enum SolverType {
        PGS,
        PJ
    };

    struct Stats {
        uint32_t static_actor_count = 0;
        uint32_t dynamic_actor_count = 0;
        uint32_t collision_count = 0;
        uint32_t contact_count = 0;
        float simulation_time = 0.f;

        MovingAverage<float> max_constraint_solver_error;
        MovingAverage<float> avg_constraint_solver_error;
    };

    struct Config {
        bool randomize_order = true;
        bool enable_gyroscopic_torque = true;
        float warmstarting_normal = 0.83f;
        float warmstarting_friction = 0.75f;
        Vec3 gravity = {0.f, -9.81f, 0.f};

        NpBackendHint np_backend_hint = NpBackendHint::GJK_EPA;

        SolverType solver_type = SolverType::PGS;
        ConstraintSolver::Config solver_config;

        // debug draw
        bool draw_contact_normals = false;
        bool draw_contact_friction = false;
    };

    explicit DynamicsWorld(std::optional<Config> init_config = std::nullopt);

    void                    add_actor(BaseActor* actor);
    void                    remove_actor(BaseActor* actor);

    void                    update(float dt);

    void                    set_debug_drawer(std::shared_ptr<DebugDrawer> drawer);
    DebugDrawer*            debug_drawer() { return m_debug_drawer.get(); }

    Config&                 config() { return m_config; }
    const Config&           config() const { return m_config; }

    const Stats&            stats() const { return m_stats; }

    uint32_t                frame_id() const { return m_frame_id; }

    Narrowphase&            narrowphase() { return m_narrowphase; }
    const Narrowphase&      narrowphase() const { return m_narrowphase; }

    ConstraintSolver&       solver() { return *m_solver; }
    const ConstraintSolver& solver() const { return *m_solver; }

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
    void apply_gyroscopic_torque(float dt);
    void perform_collision_detection();
    void collide(BaseActor& actor1, BaseActor& actor2);
    void apply_contacts();
    void cache_lambdas();
    void integrate_bodies();
    void refresh_manifolds();

    void setup_solver(SolverType type);
    void setup_narrowphase(NpBackendHint hint);

    std::unique_ptr<ConstraintSolver> m_solver;

    Vector<DynamicActor*> m_dynamic_actors;
    Vector<StaticActor*> m_static_actors;

    Vector<PendingContact> m_pending_contacts;
    Narrowphase m_narrowphase;

    UnorderedMap<ManifoldCacheKey, ManifoldCache> m_manifolds;

    std::optional<NpBackendHint> m_np_backend_hint;
    std::optional<SolverType> m_solver_type;
    Config m_config;
    Stats m_stats;

    uint32_t m_frame_id = 0;

    std::shared_ptr<DebugDrawer> m_debug_drawer;
};

} // slope
