#pragma once
#include "slope/dynamics/constraint_solver.hpp"
#include "slope/dynamics/actor.hpp"
#include "slope/dynamics/joint.hpp"
#include "slope/core/vector.hpp"
#include "slope/core/unordered_map.hpp"
#include "slope/core/array.hpp"
#include "slope/core/disjoint_set.hpp"
#include "slope/core/pair_cache.hpp"
#include "slope/collision/contact_manifold.hpp"
#include "slope/collision/narrowphase/narrowphase.hpp"
#include "slope/collision/broadphase/broadphase.hpp"
#include "slope/debug/debug_drawer.hpp"
#include "slope/thread/task_executor.hpp"
#include <memory>

namespace slope {

class DynamicsWorld {
public:
    enum class NpBackendHint {
        Mixed,
        GJK_EPA,
        SAT
    };

    struct Stats {
        Array<uint32_t, (int)ActorKind::Count> actor_count = {};
        uint32_t np_test_count = 0;
        uint32_t collision_count = 0;
        uint32_t contact_count = 0;
        float simulation_time = 0.f;

        MovingAverage<float> max_constraint_solver_error;
        MovingAverage<float> avg_constraint_solver_error;
    };

    struct Config {
        float time_interval = 0.02f;

        bool enable_gravity = true;
        bool enable_constraint_resolving = true;
        bool enable_integration = true;
        bool randomize_order = true;
        bool enable_gyroscopic_torque = true;
        bool enable_velocity_dependent_friction = true;
        bool enable_cone_friction = false;
        float warmstarting_normal = 0.83f;
        float warmstarting_friction = 0.75f;
        float warmstarting_joint = 0.8f;
        vec3 gravity = {0.f, -9.81f, 0.f};

        NpBackendHint np_backend_hint = NpBackendHint::Mixed;

        ConstraintSolver::Config solver_config;

        // debug draw
        bool draw_contact_normals1 = false;
        bool draw_contact_friction1 = false;
        bool draw_contact_normals2 = false;
        bool draw_contact_friction2 = false;
        bool delay_integration = false;
    };

    explicit DynamicsWorld(std::optional<Config> init_config = std::nullopt);

    template<class T>
    T*                      create_actor();
    // Note: any attached joint will be destroyed along with the actor
    void                    destroy_actor(BaseActor* actor);

    template<class T>
    T*                      create_joint(DynamicActor* actor1, DynamicActor* actor2 = nullptr);
    void                    destroy_joint(BaseJoint* joint);

    // Remove all actors and joints
    void                    clear();

    void                    setup_executor(TaskExecutor& executor);

    void                    set_debug_drawer(std::shared_ptr<DebugDrawer> drawer);
    DebugDrawer*            debug_drawer() { return m_debug_drawer.get(); }

    Config&                 config() { return m_config; }
    const Config&           config() const { return m_config; }

    const Stats&            stats() const { return m_stats; }

    uint32_t                frame_id() const { return m_frame_id; }

    Narrowphase&            narrowphase() { return m_narrowphase_ctx[0].narrowphase; }
    const Narrowphase&      narrowphase() const { return m_narrowphase_ctx[0].narrowphase; }

    ConstraintSolver&       solver() { return m_solve_ctx[0].solver; }
    const ConstraintSolver& solver() const { return m_solve_ctx[0].solver; }

private:

    struct ActorId {
        static constexpr int KIND_BITS = 8;
        static constexpr int KIND_MASK = (1 << KIND_BITS) - 1;

        static uint64_t compress(ActorId a, ActorId b) { return (static_cast<uint64_t>(a.raw) << 32) | b.raw; }

        ActorId(ActorKind kind, uint32_t index) : raw((index << KIND_BITS) | ((int)kind)) {}

        ActorKind   kind() const { return static_cast<ActorKind>(raw & KIND_MASK); }
        uint32_t    index() const { return raw >> KIND_BITS; }

        uint32_t    raw;
    };

    using Broadphase = Broadphase<ActorId>;

    struct ActorData {
        std::unique_ptr<BaseActor>  actor;
        Broadphase::ProxyId         proxy_id = 0;
    };

    struct ManifoldCache {
        ContactManifold manifold;
        DynamicActor* actor1 = nullptr;
        BaseActor* actor2 = nullptr;
        uint32_t touch_frame_id = 0;
    };

    struct PendingContact {
        const ManifoldCache* mf_cache;
        ManifoldPoint* mf_point;
        uint32_t actor_index;
    };

    struct BroadphaseContext {
        Vector<Broadphase::Overlap> overlaps;
    };

    struct NarrowphaseContext {
        Narrowphase                             narrowphase;
        NpContactPatch                          contact_patch;
        Vector<PendingContact>                  pending_contacts;
        Vector<std::pair<uint32_t, uint32_t>>   actor_contacts;
        PairCache<ManifoldCache>                new_manifolds;
    };

    struct SolveContext {
        ConstraintSolver solver;
        Vector<PendingContact>  pending_contacts;
        ConstraintIds normal_range;
        ConstraintIds friction_range;
    };

    struct Island {
        uint32_t root;
        uint32_t size;
    };

    void setup_collision_detection(TaskExecutor& executor, TaskId pre_fence, TaskId post_fence);

    void merge_contacts();
    void randomize_contacts(int solver_id);
    void allocate_constraints();
    void apply_contacts(int worker_id, int concurrency, int solver_id);
    void apply_external_forces();
    void collide(NarrowphaseContext& ctx, ActorId actor1_id, ActorId actor2_id);

    void apply_joints();
    void update_constraint_stats();
    void update_general_stats();
    void cache_lambdas(int worker_id, int concurrency);
    void integrate_bodies(int worker_id, int concurrency);
    void refresh_manifolds();

    void setup_solver();
    void setup_narrowphase();
    void reset_frame_stats();

    // TODO: optimize storage
    Array<Vector<ActorData>, (int)ActorKind::Count> m_actors;
    Vector<std::unique_ptr<BaseJoint>>              m_joints;

    Vector<BroadphaseContext>       m_broadphase_ctx;
    Broadphase                      m_broadphase;
    Vector<Broadphase::Overlap>     m_overlaps;

    Vector<NarrowphaseContext>      m_narrowphase_ctx;
    NpBackendHint                   m_np_backend_hint;
    PairCache<ManifoldCache>        m_manifolds;

    DisjointSet                     m_islands_ds;
    Vector<Island>                  m_islands;
    Vector<uint32_t>                m_island_to_bin;

    Vector<SolveContext>            m_solve_ctx;

    Config                          m_config;
    Stats                           m_stats;
    uint32_t                        m_frame_id = 0;

    std::shared_ptr<DebugDrawer>    m_debug_drawer;
};

template<class T>
T* DynamicsWorld::create_actor()
{
    static_assert(std::is_base_of_v<BaseActor, T>);

    auto actor_ptr = std::make_unique<T>();
    auto* actor = actor_ptr.get();

    auto& container = m_actors[(int)actor->kind()];
    auto proxy_id = m_broadphase.add_proxy({actor->kind(), static_cast<uint32_t>(container.size())});

    container.push_back({std::move(actor_ptr), proxy_id});

    return actor;
}

template<class T>
T* DynamicsWorld::create_joint(DynamicActor* actor1, DynamicActor* actor2)
{
    static_assert(std::is_base_of_v<BaseJoint, T>);

    auto* body1 = &actor1->body();
    auto* body2 = actor2 ? &actor2->body() : nullptr;
    auto joint_ptr = std::make_unique<T>(body1, body2);
    T* joint = joint_ptr.get();
    m_joints.push_back(std::move(joint_ptr));

    return joint;
}

} // slope
