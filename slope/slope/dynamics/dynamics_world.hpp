#pragma once
#include "slope/dynamics/constraint_solver.hpp"
#include "slope/dynamics/actor.hpp"
#include "slope/containers/vector.hpp"
#include "slope/containers/unordered_map.hpp"
#include "slope/collision/contact_manifold.hpp"
#include "slope/collision/narrowphase/narrowphase.hpp"
#include "slope/collision/broadphase/broadphase.hpp"
#include "slope/debug/debug_drawer.hpp"
#include <memory>

namespace slope {
using ManifoldCacheKey = std::pair<const void*, const void*>;
}

namespace std
{
template<> struct hash<slope::ManifoldCacheKey>
{
    std::size_t operator()(const slope::ManifoldCacheKey& key) const noexcept
    {
        auto a = reinterpret_cast<size_t>(key.first);
        auto b = reinterpret_cast<size_t>(key.second);
        return a ^ (b + 0x9e3779b9 + (a << 6) + (a >> 2));
    }
};
}

namespace slope {

class DynamicsWorld {
public:
    enum class NpBackendHint {
        Mixed,
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
        uint32_t np_test_count = 0;
        uint32_t collision_count = 0;
        uint32_t contact_count = 0;
        float simulation_time = 0.f;

        MovingAverage<float> max_constraint_solver_error;
        MovingAverage<float> avg_constraint_solver_error;
    };

    struct Config {
        bool enable_gravity = true;
        bool enable_constraint_resolving = true;
        bool enable_integration = true;
        bool randomize_order = true;
        bool enable_gyroscopic_torque = true;
        bool enable_velocity_dependent_friction = true;
        bool enable_cone_friction = false;
        float warmstarting_normal = 0.83f;
        float warmstarting_friction = 0.75f;
        Vec3 gravity = {0.f, -9.81f, 0.f};

        NpBackendHint np_backend_hint = NpBackendHint::Mixed;

        SolverType solver_type = SolverType::PGS;
        ConstraintSolver::Config solver_config;

        // debug draw
        bool draw_contact_normals1 = false;
        bool draw_contact_friction1 = false;
        bool draw_contact_normals2 = false;
        bool draw_contact_friction2 = false;
        bool delay_integration = false;
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
        DynamicActor* actor1 = nullptr;
        BaseActor* actor2 = nullptr;
        uint32_t touch_frame_id = 0;
    };

    struct PendingContact {
        const ManifoldCache* mf_cache;
        ManifoldPoint* mf_point;
    };

    template <class Actor>
    struct ActorData {
        Actor* actor = nullptr;
        Broadphase<BaseActor>::ProxyId proxy_id = 0;
    };

    template<class T>
    void remove_actor_impl(Vector<T>& container, BaseActor* actor);

    void apply_gravity();
    void apply_gyroscopic_torque(float dt);
    void perform_collision_detection();
    void collide(BaseActor* actor1, BaseActor* actor2);
    void apply_contacts();
    void update_constraint_stats();
    void update_general_stats();
    void cache_lambdas();
    void integrate_bodies();
    void refresh_manifolds();

    void setup_solver(SolverType type, float dt);
    void setup_narrowphase(NpBackendHint hint);

    std::unique_ptr<ConstraintSolver> m_solver;

    Vector<ActorData<DynamicActor>> m_dynamic_actors;
    Vector<ActorData<StaticActor>> m_static_actors;

    Broadphase<BaseActor> m_broadphase;

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
