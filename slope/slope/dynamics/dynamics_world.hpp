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

    StaticActor*            create_static_actor();
    DynamicActor*           create_dynamic_actor();
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

    Narrowphase&            narrowphase() { return *m_narrowphase_ctx[0].narrowphase; }
    const Narrowphase&      narrowphase() const { return *m_narrowphase_ctx[0].narrowphase; }

    ConstraintSolver&       solver() { return m_solve_ctx[0].solver; }
    const ConstraintSolver& solver() const { return m_solve_ctx[0].solver; }

private:
    using Broadphase = Broadphase<BaseActor*>;

    struct JointData {
        std::unique_ptr<BaseJoint>  joint;
        BaseActor*                  actor1 = nullptr;
        BaseActor*                  actor2 = nullptr;
    };

    struct ManifoldCache {
        ContactManifold manifold;
        DynamicActor* actor1 = nullptr;
        BaseActor* actor2 = nullptr;
        uint32_t touch_frame_id = 0;
    };

    struct PendingContact {
        const ManifoldCache*    mf_cache;
        ManifoldPoint*          mf_point;
        BaseActor::LinearId     actor1_linear_id;
    };

    struct BroadphaseContext {
        Vector<Broadphase::Overlap> overlaps;
    };

    struct NarrowphaseContext {
        using ContactPair = std::pair<BaseActor::LinearId, BaseActor::LinearId>;

        std::unique_ptr<Narrowphase>    narrowphase;
        NpContactPatch                  contact_patch;
        Vector<PendingContact>          pending_contacts;
        Vector<ContactPair>             actor_contacts;
        PairCache<ManifoldCache>        new_manifolds;
    };

    struct SolveContext {
        ConstraintSolver        solver;
        Vector<PendingContact>  pending_contacts;
        Vector<BaseJoint*>      pending_joints;
        ConstraintId            normal_begin_id;
        ConstraintId            friction_begin_id;
    };

    struct Island {
        BaseActor::LinearId root;
        uint32_t            size;
    };

    template <class T>
    T* create_actor_impl(Vector<std::unique_ptr<T>>& container);

    template <class T>
    void destroy_actor_impl(Vector<std::unique_ptr<T>>& container, BaseActor* actor);

    void prepare_to_update();

    void setup_collision_detection(TaskExecutor& executor, TaskId pre_fence, TaskId post_fence);

    void merge_joints();
    void apply_joints(int worker_id);

    void merge_islands();
    void randomize_contacts(int solver_id);
    void allocate_constraints();
    void apply_contacts(int worker_id, int concurrency, int solver_id);
    void apply_external_forces();
    void collide(NarrowphaseContext& ctx, BaseActor* actor1, BaseActor* actor2);

    void update_constraint_stats();
    void update_general_stats();
    void cache_lambdas(int worker_id, int concurrency);
    void integrate_bodies(int worker_id, int concurrency);
    void refresh_manifolds();

    void finalize_update();

    void setup_solver();
    void setup_narrowphase();
    void reset_frame_stats();

    // TODO: optimize storage
    Vector<std::unique_ptr<StaticActor>>    m_static_actors;
    Vector<std::unique_ptr<DynamicActor>>   m_dynamic_actors;
    Vector<JointData>                       m_joints;

    Vector<BroadphaseContext>       m_broadphase_ctx;
    Broadphase                      m_broadphase;
    Vector<Broadphase::Overlap>     m_overlaps;

    Vector<NarrowphaseContext>      m_narrowphase_ctx;
    NpBackendHint                   m_np_backend_hint;
    PairCache<ManifoldCache>        m_manifolds;

    DisjointSet                     m_island_merger;
    Vector<Island>                  m_islands;
    Vector<uint32_t>                m_island_to_bin;

    Vector<SolveContext>            m_solve_ctx;

    Config                          m_config;
    Stats                           m_stats;
    uint32_t                        m_frame_id = 0;

    std::shared_ptr<DebugDrawer>    m_debug_drawer;
};

template<class T>
T* DynamicsWorld::create_joint(DynamicActor* actor1, DynamicActor* actor2)
{
    static_assert(std::is_base_of_v<BaseJoint, T>);

    auto* body1 = &actor1->body();
    auto* body2 = actor2 ? &actor2->body() : nullptr;
    auto joint_ptr = std::make_unique<T>(body1, body2);
    T* joint = joint_ptr.get();

    m_joints.push_back({ std::move(joint_ptr), actor1, actor2 });

    return joint;
}

} // slope
