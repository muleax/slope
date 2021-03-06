#pragma once
#include "slope/dynamics/constraint_solver.hpp"
#include "slope/dynamics/actor.hpp"
#include "slope/dynamics/joint/base_joint.hpp"
#include "slope/core/vector.hpp"
#include "slope/core/unordered_map.hpp"
#include "slope/core/array.hpp"
#include "slope/core/disjoint_set.hpp"
#include "slope/core/pair_cache.hpp"
#include "slope/collision/contact_manifold.hpp"
#include "slope/collision/narrowphase/narrowphase.hpp"
#include "slope/collision/broadphase/broadphase.hpp"
#include "slope/debug/debug_drawer.hpp"
#include "slope/flow/task_executor.hpp"
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
    vec3    gravity = {0.f, -9.81f, 0.f};
    bool    enable_constraint_resolving = true;
    bool    enable_integration = true;
    bool    randomize_order = true;
    bool    enable_gyroscopic_torque = true;
    bool    enable_velocity_dependent_friction = true;
    bool    enable_cone_friction = false;
    float   normal_warmstarting = 0.8f;
    float   friction_warmstarting = 0.75f;
    float   contact_erp = 0.2f;
    float   contact_cfm = 0.f;
    float   contact_penetration = 0.01f;
    float   joint_warmstarting = 0.8f;
    float   joint_erp = 0.15f;

    // debug draw
    bool    draw_contact_normals1 = false;
    bool    draw_contact_friction1 = false;
    bool    draw_contact_normals2 = false;
    bool    draw_contact_friction2 = false;
    bool    delay_integration = false;
};

struct DynamicsWorldStats {
    float               simulation_time = 0.f;
    uint32_t            kinematic_actor_count = 0;
    uint32_t            dynamic_actor_count = 0;
    uint32_t            joint_count = 0;
    uint32_t            island_count = 0;
    Array<uint32_t, 4>  larges_islands = {};
    uint32_t            constraint_count = 0;

    NarrowphaseStats    np_stats;

    void reset() { np_stats.reset(); }
};

class DynamicsWorld : public ConfigHolder<DynamicsWorldConfig> {
public:
    explicit DynamicsWorld(std::optional<DynamicsWorldConfig> init_config = std::nullopt);

    void                    setup_executor(TaskExecutor& executor);

    KinematicActor*         create_kinematic_actor();
    DynamicActor*           create_dynamic_actor();
    // Note: any attached joint will be destroyed along with the actor
    void                    destroy_actor(BaseActor* actor);

    template<class Shape>
    void                    assign_shape(BaseActor* actor, Shape&& shape);

    template<class Joint>
    Joint*                  create_joint(DynamicActor* actor1, DynamicActor* actor2 = nullptr);
    void                    destroy_joint(BaseJoint* joint);

    // Remove all actors and joints
    void                    clear();

    void                    set_debug_drawer(std::shared_ptr<DebugDrawer> drawer);
    DebugDrawer*            debug_drawer() { return m_debug_drawer.get(); }

    void                    clear_stats();
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
        Vector<ContactPair>             actor_pairs;
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

    struct Bin {
        uint32_t index = 0;
        uint32_t size = 0;

        // for min-heap
        bool operator<(const Bin& other) const { return size > other.size; }
    };

    template <class Actor>
    struct ActorData {
        std::unique_ptr<Actor>          actor;
        std::unique_ptr<CollisionShape> shape;
    };

    void on_config_update(const DynamicsWorldConfig& prev_config) final;

    void reset_stats();
    void finalize_stats();

    template <class Actor>
    Actor* create_actor_impl(Vector<ActorData<Actor>>& container);

    template <class Actor>
    void destroy_actor_impl(Vector<ActorData<Actor>>& container, BaseActor* actor);

    template <class Actor, class Shape>
    void set_shape_impl(Vector<ActorData<Actor>>& container, BaseActor* actor, Shape&& shape);

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
    Vector<ActorData<KinematicActor>>   m_kinematic_actors;
    Vector<ActorData<DynamicActor>>     m_dynamic_actors;
    Vector<JointData>                   m_joints;

    Vector<BroadphaseContext>       m_broadphase_ctx;
    Broadphase                      m_broadphase;
    Vector<Broadphase::Overlap>     m_overlaps;

    Vector<NarrowphaseContext>      m_narrowphase_ctx;
    PairCache<ManifoldCache>        m_manifolds;

    DisjointSet                     m_island_merger;
    Vector<Island>                  m_islands;
    Vector<uint32_t>                m_island_to_bin;
    Vector<Bin>                     m_bin_heap;

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

template<class Shape>
void DynamicsWorld::assign_shape(BaseActor* actor, Shape&& shape)
{
    if (actor->is<DynamicActor>())
        set_shape_impl(m_dynamic_actors, actor, std::forward<Shape>(shape));
    else
        set_shape_impl(m_kinematic_actors, actor, std::forward<Shape>(shape));
}

template <class Actor, class Shape>
void DynamicsWorld::set_shape_impl(Vector<ActorData<Actor>>& container, BaseActor* actor, Shape&& shape)
{
    auto& data = container[actor->linear_id()];
    data.shape = std::make_unique<std::remove_reference_t<Shape>>(std::forward<Shape>(shape));
    actor->assign_shape(data.shape.get());
}

} // slope
