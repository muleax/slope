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

enum class NpBackendHint {
    Mixed,
    GJK_EPA,
    SAT
};

struct DynamicsWorldConfig {
    NpBackendHint           np_backend_hint = NpBackendHint::Mixed;
    NarrowphaseConfig       np_config;

    ConstraintSolverConfig  solver_config;

    bool    enable_gravity = true;
    bool    enable_constraint_resolving = true;
    bool    enable_integration = true;
    bool    randomize_order = true;
    bool    enable_gyroscopic_torque = true;
    bool    enable_velocity_dependent_friction = true;
    bool    enable_cone_friction = false;
    float   warmstarting_normal = 0.83f;
    float   warmstarting_friction = 0.75f;
    float   warmstarting_joint = 0.8f;
    vec3    gravity = {0.f, -9.81f, 0.f};

    // debug draw
    bool    draw_contact_normals1 = false;
    bool    draw_contact_friction1 = false;
    bool    draw_contact_normals2 = false;
    bool    draw_contact_friction2 = false;
    bool    delay_integration = false;
};

struct DynamicsWorldStats {
    uint32_t    static_actor_count = 0;
    uint32_t    dynamic_actor_count = 0;
    float       simulation_time = 0.f;

    NarrowphaseStats np_stats;

    void reset() { np_stats.reset(); }
};

class DynamicsWorld : public ConfigHolder<DynamicsWorldConfig> {
public:
    explicit DynamicsWorld(std::optional<DynamicsWorldConfig> init_config = std::nullopt);

    void                    setup_executor(TaskExecutor& executor);

    StaticActor*            create_static_actor();
    DynamicActor*           create_dynamic_actor();
    // Note: any attached joint will be destroyed along with the actor
    void                    destroy_actor(BaseActor* actor);

    template<class T>
    T*                      create_joint(DynamicActor* actor1, DynamicActor* actor2 = nullptr);
    void                    destroy_joint(BaseJoint* joint);

    // Remove all actors and joints
    void                    clear();

    void                    set_debug_drawer(std::shared_ptr<DebugDrawer> drawer);
    DebugDrawer*            debug_drawer() { return m_debug_drawer.get(); }

    const auto&             stats() const { return m_stats; }

    uint32_t                frame_id() const { return m_frame_id; }

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

    void on_config_update(const DynamicsWorldConfig& prev_config) final;

    void reset_stats();
    void finalize_stats();

    template <class T>
    T* create_actor_impl(Vector<std::unique_ptr<T>>& container);

    template <class T>
    void destroy_actor_impl(Vector<std::unique_ptr<T>>& container, BaseActor* actor);

    void reset_narrowphase_backend(NpBackendHint hint);

    void setup_flow(TaskExecutor& executor);
    void setup_collision_flow(TaskExecutor& executor, TaskId pre_fence, TaskId post_fence);
    void setup_apply_constraints_flow(TaskExecutor& executor, TaskId pre_fence, TaskId post_fence);
    void setup_solve_flow(TaskExecutor& executor, TaskId pre_fence, TaskId post_fence);
    void setup_post_solve_flow(TaskExecutor& executor, TaskId pre_fence);

    void prepare_to_update();
    void apply_external_forces();
    void collide(NarrowphaseContext& ctx, BaseActor* actor1, BaseActor* actor2);
    void merge_joints();
    void merge_islands();
    void allocate_constraints();
    void randomize_contacts(int solver_id);
    void apply_contacts(int worker_id, int concurrency, int solver_id);
    void apply_joints(int worker_id);

    void cache_lambdas(int worker_id, int concurrency);
    void integrate_bodies(int worker_id, int concurrency);
    void refresh_manifolds();
    void debug_draw();
    void update_general_stats();
    void finalize_update();

    // TODO: optimize storage
    Vector<std::unique_ptr<StaticActor>>    m_static_actors;
    Vector<std::unique_ptr<DynamicActor>>   m_dynamic_actors;
    Vector<JointData>                       m_joints;

    Vector<BroadphaseContext>       m_broadphase_ctx;
    Broadphase                      m_broadphase;
    Vector<Broadphase::Overlap>     m_overlaps;

    Vector<NarrowphaseContext>      m_narrowphase_ctx;
    PairCache<ManifoldCache>        m_manifolds;

    DisjointSet                     m_island_merger;
    Vector<Island>                  m_islands;
    Vector<uint32_t>                m_island_to_bin;

    Vector<SolveContext>            m_solve_ctx;

    uint32_t                        m_frame_id = 0;

    DynamicsWorldStats              m_stats;

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
