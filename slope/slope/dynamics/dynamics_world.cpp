#include "slope/dynamics/dynamics_world.hpp"
#include "slope/dynamics/constraint_solver.hpp"
#include "slope/collision/narrowphase/polyhedra_backends.hpp"
#include "slope/collision/narrowphase/capsule_backends.hpp"
#include "slope/collision/narrowphase/sphere_backends.hpp"
#include "slope/debug/log.hpp"
#include <optional>

namespace slope {

namespace {

struct FrictionBasis {
    vec3 axis1;
    vec3 axis2;
};

std::optional<FrictionBasis> get_velocity_dependent_friction_basis(
    RigidBody* body1, RigidBody* body2, ConstraintGeom& geom, float threshold)
{
    std::optional<FrictionBasis> basis;

    vec3 vel = body1->point_velocity(geom.p1);
    if (body2)
        vel -= body2->point_velocity(geom.p2);

    auto lat_vel = vel - geom.axis * geom.axis.dot(vel);
    if (lat_vel.length_squared() > threshold * threshold) {
        auto axis1 = lat_vel.normalized();
        auto axis2 = geom.axis.cross(axis1);
        basis = FrictionBasis{axis1, axis2};
    }

    return basis;
}

} // unnamed

DynamicsWorld::DynamicsWorld(std::optional<Config> init_config)
{
    // TODO: remove
    m_solve_ctx.resize(1);
    m_broadphase_ctx.resize(1);
    m_narrowphase_ctx.resize(1);

    if (init_config)
        m_config = *init_config;

    setup_narrowphase();
    setup_solver();
}

void DynamicsWorld::setup_solver()
{
    for (auto& ctx: m_solve_ctx) {
        ctx.solver.config() = m_config.solver_config;
        ctx.solver.set_time_interval(m_config.time_interval);
    }
}

void DynamicsWorld::setup_narrowphase()
{
    m_np_backend_hint = m_config.np_backend_hint;

    for (auto& ctx: m_narrowphase_ctx) {
        auto& narrowphase = ctx.narrowphase;
        narrowphase.reset_all_backends();

        if (m_np_backend_hint == NpBackendHint::Mixed) {
            narrowphase.add_backend<GJKConvexPolyhedronBackend>();
            narrowphase.add_backend<GJKConvexPolyhedronBoxBackend>();
            narrowphase.add_backend<SATBoxBackend>();

        } else if (m_np_backend_hint == NpBackendHint::GJK_EPA) {
            narrowphase.add_backend<GJKConvexPolyhedronBackend>();
            narrowphase.add_backend<GJKConvexPolyhedronBoxBackend>();
            narrowphase.add_backend<GJKBoxBackend>();

        } else {
            narrowphase.add_backend<SATConvexPolyhedronBackend>();
            narrowphase.add_backend<SATConvexPolyhedronBoxBackend>();
            narrowphase.add_backend<SATBoxBackend>();
        }

        narrowphase.add_backend<ConvexPolyhedronCapsuleBackend>();
        narrowphase.add_backend<ConvexPolyhedronSphereBackend>();

        narrowphase.add_backend<BoxCapsuleBackend>();
        narrowphase.add_backend<BoxSphereBackend>();

        narrowphase.add_backend<CapsuleSphereBackend>();
        narrowphase.add_backend<CapsuleBackend>();

        narrowphase.add_backend<SphereBackend>();
    }
}

void DynamicsWorld::destroy_actor(BaseActor* actor)
{
    auto& actors = m_actors[(int)actor->kind()];
    // TODO: optimize
    auto it = std::find_if(actors.begin(), actors.end(),
                           [&actor](auto& data) { return data.actor.get() == actor; });
    if (it == actors.end()) {
        log::error("Destroy actor: actor not found");
        return;
    }

    m_broadphase.remove_proxy(it->proxy_id);

    if (actor->is<DynamicActor>()) {
        auto* body = &actor->cast<DynamicActor>()->body();
        // remove attached joints
        // TODO: optimize
        auto joint_it = std::remove_if(m_joints.begin(), m_joints.end(),
                                 [body](auto& j) { return j->body1() == body || j->body2() == body; });
        m_joints.erase(joint_it, m_joints.end());
    }

    std::swap(*it, actors.back());
    actors.pop_back();

    m_broadphase.remove_proxy(it->proxy_id);
    it->proxy_id = m_broadphase.add_proxy({it->actor->kind(), static_cast<uint32_t>(it - actors.begin())});
}

void DynamicsWorld::destroy_joint(BaseJoint* joint)
{
    // TODO: optimize
    auto it = std::remove_if(m_joints.begin(), m_joints.end(),
                             [joint](auto& j) { return j.get() == joint; });
    m_joints.erase(it, m_joints.end());
}

void DynamicsWorld::clear()
{
    m_broadphase.clear();

    for (auto& actors : m_actors)
        actors.clear();

    m_joints.clear();
}

void DynamicsWorld::set_debug_drawer(std::shared_ptr<DebugDrawer> drawer)
{
    m_debug_drawer = std::move(drawer);
}

void DynamicsWorld::apply_external_forces()
{
    float dt = m_config.time_interval;
    for (auto& data: m_actors[(int) ActorKind::Dynamic]) {
        auto* actor = data.actor->cast<DynamicActor>();
        actor->body().apply_force_to_com(m_config.gravity * actor->body().mass());
        actor->body().apply_gyroscopic_torque(dt);
    }
}

void DynamicsWorld::collide(NarrowphaseContext& ctx, ActorId actor1_id, ActorId actor2_id)
{
    if (actor1_id.kind() != ActorKind::Dynamic) {
        if (actor2_id.kind() != ActorKind::Dynamic) {
            // ignore static to static collisions
            // TODO: filter out this case earlier in broadphase
            return;
        } else {
            std::swap(actor1_id, actor2_id);
        }
    }

    BaseActor* actor1 = m_actors[(int)actor1_id.kind()][actor1_id.index()].actor.get();
    BaseActor* actor2 = m_actors[(int)actor2_id.kind()][actor2_id.index()].actor.get();

    auto& shape1 = actor1->shape();
    auto& shape2 = actor2->shape();

    //m_stats.np_test_count++;

    ctx.contact_patch.reset();

    if (ctx.narrowphase.collide(&shape1, &shape2, ctx.contact_patch)) {
        //m_stats.collision_count++;

        ManifoldCache* cache = nullptr;

        PairCacheKey cache_key = {&shape1, &shape2};
        auto it = m_manifolds.find(cache_key);
        if (it != m_manifolds.end()) {
            cache = &it->second;
        } else {
            cache = &ctx.new_manifolds[cache_key];
        }

        // TODO: check in broadphase
        cache->actor1 = actor1->cast<DynamicActor>();
        cache->actor2 = actor2;
        cache->touch_frame_id = m_frame_id;

        if (actor2->is<DynamicActor>())
            ctx.actor_contacts.push_back({ actor1_id.index(), actor2_id.index() });

        auto& manifold = cache->manifold;
        manifold.update_inv_transform(actor1->inv_transform());
        manifold.begin_update();

        ctx.contact_patch.normalize_order();
        for (auto& geom : ctx.contact_patch.contacts)
            manifold.add_contact(geom);

        manifold.end_update();

        auto& pending_contacts = ctx.pending_contacts;
        for (auto& p : manifold) {
            pending_contacts.push_back({cache, &p, actor1_id.index()});
        }
    }
}

void DynamicsWorld::apply_contacts(int worker_id, int concurrency, int solver_id)
{
    auto& ctx = m_solve_ctx[solver_id];

    size_t chunk_size = ctx.pending_contacts.size() / concurrency;

    int chunk_beg = worker_id * chunk_size;
    int chunk_end = (worker_id == concurrency - 1)
                    ? ctx.pending_contacts.size()
                    : chunk_beg + chunk_size;

    ConstraintId normal_id{ctx.normal_range.first.group(), ctx.normal_range.first.index() + chunk_beg};
    ConstraintId friction_id{ctx.friction_range.first.group(),
                             ctx.friction_range.first.index() + chunk_beg * 2};

    for (auto it = ctx.pending_contacts.begin() + chunk_beg;
         it != ctx.pending_contacts.begin() + chunk_end; ++it) {
        auto[mf_cache, mf_point, _] = *it;

        ConstraintGeom geom = {mf_point->geom.p1, mf_point->geom.p2, mf_point->geom.normal};

        auto* actor1 = mf_cache->actor1;
        auto* actor2 = mf_cache->actor2;

        SL_ASSERT(actor1->is<DynamicActor>());
        RigidBody* body1 = &actor1->cast<DynamicActor>()->body();
        RigidBody* body2 = actor2->is<DynamicActor>() ? &actor2->cast<DynamicActor>()->body() : nullptr;

        std::optional<FrictionBasis> basis;
        if (m_config.enable_velocity_dependent_friction) {
            // TODO: reconsider threshold
            basis = get_velocity_dependent_friction_basis(body1, body2, geom, 0.3f);
        }

        if (!basis) {
            basis.emplace();
            find_tangent(basis->axis1, basis->axis2, geom.axis);
        }

        // TODO: implement policies
        float friction_ratio = actor1->friction() * actor2->friction();

        auto fc1 = Constraint::bilateral(body1, body2, {geom.p1, geom.p2, basis->axis1});
        auto fc2 = Constraint::bilateral(body1, body2, {geom.p1, geom.p2, basis->axis2});

        fc1.init_lambda = mf_point->friction1_lambda * m_config.warmstarting_friction;
        fc2.init_lambda = mf_point->friction2_lambda * m_config.warmstarting_friction;

        auto nc = Constraint::stabilized_unilateral(body1, body2, geom);
        nc.init_lambda = mf_point->normal_lambda * m_config.warmstarting_normal;
        ctx.solver.setup_constraint(normal_id, nc);
        mf_point->normal_constr_id = normal_id;

        ++normal_id;

        ConstraintIds friction_ids;
        friction_ids.first = friction_id;
        ++friction_id;
        friction_ids.second = friction_id;
        ++friction_id;

        mf_point->friction1_constr_id = friction_ids.first;
        mf_point->friction2_constr_id = friction_ids.second;

        if (m_config.enable_cone_friction) {
            ctx.solver.setup_friction_cone(friction_ids, fc1, fc2, {friction_ratio, friction_ratio},
                                           mf_point->normal_constr_id);

        } else {
            ctx.solver.setup_friction_2d(friction_ids, fc1, fc2, {friction_ratio, friction_ratio},
                                         mf_point->normal_constr_id);
        }
    }

/*
 * auto* debug_drawer = m_debug_drawer.get();
            if (debug_drawer) {
                auto& geom = p->geom;

                if (m_config.draw_contact_normals1) {
                    debug_drawer->draw_line(geom.p1, geom.p1 + geom.normal * 0.3f, {0.2f, 1.f, 0.2f});
                }

                if (m_config.draw_contact_friction1) {
                    debug_drawer->draw_line(geom.p1, geom.p1 + fc1.jacobian1[0] * 0.3f, {1.f, 0.1f, 0.2f});
                    debug_drawer->draw_line(geom.p1, geom.p1 + fc2.jacobian1[0] * 0.3f, {1.f, 0.1f, 0.2f});
                }

                if (m_config.draw_contact_normals2) {
                    debug_drawer->draw_line(geom.p2, geom.p2 - geom.normal * 0.3f, {0.2f, 1.f, 0.2f});
                }

                if (m_config.draw_contact_friction2) {
                    debug_drawer->draw_line(geom.p2, geom.p2 + fc1.jacobian1[0] * 0.3f, {1.f, 0.1f, 0.2f});
                    debug_drawer->draw_line(geom.p2, geom.p2 + fc2.jacobian1[0] * 0.3f, {1.f, 0.1f, 0.2f});
                }
            }
            */
}

void DynamicsWorld::randomize_contacts(int solver_id)
{
    if (!m_config.randomize_order)
        return;

    auto& ctx = m_solve_ctx[solver_id];
    for (auto& c: ctx.pending_contacts) {
        auto r = rand() % ctx.pending_contacts.size();
        std::swap(c, ctx.pending_contacts[r]);
    }
}

void DynamicsWorld::allocate_constraints()
{
    if (!m_config.enable_constraint_resolving)
        return;

    for (auto& ctx : m_solve_ctx) {
        int contact_count = ctx.pending_contacts.size();
        ctx.normal_range = ctx.solver.allocate_range(ConstraintGroup::General, contact_count);
        ConstraintGroup friction_group = m_config.enable_cone_friction
                                         ? ConstraintGroup::FrictionCone
                                         : ConstraintGroup::Friction2D;
        ctx.friction_range = ctx.solver.allocate_range(friction_group, contact_count * 2);
    }
}

void DynamicsWorld::apply_joints()
{
    //TODO
    for (auto& joint : m_joints) {
        joint->set_warmstarting_ratio(m_config.warmstarting_joint);
        //joint->apply_constraints(m_solver.get());
    }
}

void DynamicsWorld::cache_lambdas(int worker_id, int concurrency)
{
    for (auto& ctx : m_solve_ctx) {
        auto [chunk_beg, chunk_end] = select_sequence_chunk(worker_id, concurrency, ctx.pending_contacts);

        for (auto it = chunk_beg; it != chunk_end; it++) {
            auto* p = it->mf_point;
            p->normal_lambda = ctx.solver.get_lambda(p->normal_constr_id);
            p->friction1_lambda = ctx.solver.get_lambda(p->friction1_constr_id);
            p->friction2_lambda = ctx.solver.get_lambda(p->friction2_constr_id);
        }
    }

    for (auto& joint : m_joints) {
        //joint->cache_lambdas(m_solver.get());
    }
}

void DynamicsWorld::integrate_bodies(int worker_id, int concurrency)
{
    auto& dyn_actors = m_actors[(int)ActorKind::Dynamic];
    auto [chunk_beg, chunk_end] = select_sequence_chunk(worker_id, concurrency, dyn_actors);

    for (auto it = chunk_beg; it != chunk_end; it++) {
        auto* actor = it->actor->cast<DynamicActor>();
        auto& body = actor->body();
        body.integrate(m_config.time_interval);
        actor->shape().set_transform(body.transform());
    }
}

void DynamicsWorld::refresh_manifolds()
{
    for (auto it = m_manifolds.begin(); it != m_manifolds.end();) {
        auto& cache = it->second;
        if (cache.touch_frame_id == m_frame_id && cache.manifold.size() > 0)
            ++it;
        else
            it = m_manifolds.erase(it);
    }

    for (auto& ctx : m_narrowphase_ctx) {
        m_manifolds.insert(ctx.new_manifolds.begin(), ctx.new_manifolds.end());
        ctx.new_manifolds.clear();
    }

    m_frame_id++;
}

void DynamicsWorld::reset_frame_stats()
{
    m_stats.np_test_count = 0;
    m_stats.collision_count = 0;

    for (auto& ctx : m_narrowphase_ctx) {
        ctx.narrowphase.gjk_solver().reset_stats();
        ctx.narrowphase.epa_solver().reset_stats();
        ctx.narrowphase.sat_solver().reset_stats();
    }
}

void DynamicsWorld::update_constraint_stats()
{
}

void DynamicsWorld::update_general_stats()
{
    for (int kind = 0; kind < (int)ActorKind::Count; kind++)
        m_stats.actor_count[kind] = m_actors[kind].size();

    m_stats.simulation_time += m_config.time_interval;
}

void DynamicsWorld::merge_contacts()
{
    auto& dyn_actors = m_actors[(int) ActorKind::Dynamic];
    m_islands_ds.reset(dyn_actors.size());
    m_island_to_bin.resize(dyn_actors.size());
    m_islands.clear();

    for (auto& ctx : m_narrowphase_ctx) {
        for (auto[a, b]: ctx.actor_contacts) {
            m_islands_ds.merge(a, b);
        }

        ctx.actor_contacts.clear();
    }

    for (uint32_t i = 0; i < dyn_actors.size(); i++) {
        if (i == m_islands_ds.find_root(i)) {
            m_islands.push_back({i, m_islands_ds.get_size(i)});
        }
    }

    std::sort(m_islands.begin(), m_islands.end(), [](auto& a, auto& b) { return a.size > b.size; } );

    int solve_concurrency = static_cast<int>(m_solve_ctx.size());
    if (solve_concurrency == 1) {
        for (auto& island : m_islands) {
            m_island_to_bin[island.root] = 0;
        }
    } else {
        // TODO: implement optimal packing
        int bin_cnt = 0;
        for (auto& island : m_islands) {
            m_island_to_bin[island.root] = bin_cnt % solve_concurrency;
            bin_cnt++;
        }
    }

    for (uint32_t i = 0; i < dyn_actors.size(); i++) {
        uint32_t root = m_islands_ds.find_root(i);
        uint32_t bin = m_island_to_bin[root];
        auto& body = dyn_actors[i].actor->cast<DynamicActor>()->body();
        m_solve_ctx[bin].solver.register_body(&body);
    }

    for (auto& ctx : m_narrowphase_ctx) {
        for (auto& c : ctx.pending_contacts) {
            uint32_t root = m_islands_ds.find_root(c.actor_index);
            uint32_t bin = m_island_to_bin[root];
            m_solve_ctx[bin].pending_contacts.push_back(c);
        }
        ctx.pending_contacts.clear();
    }
}

void DynamicsWorld::setup_collision_detection(TaskExecutor& executor, TaskId pre_fence, TaskId post_fence)
{
    int concurrency = executor.concurrency();

    int bp_concurrency = (concurrency > 1) ? concurrency * 4 : 1;
    m_broadphase_ctx.resize(bp_concurrency);
    m_broadphase.set_concurrency(bp_concurrency);

    int np_concurrency = (concurrency > 1) ? concurrency * 4 : 1;
    m_narrowphase_ctx.resize(np_concurrency);
    setup_narrowphase();

    TaskId prepare_broadphase = executor.emplace([this](){
        for (auto& actors : m_actors)
            for (auto& data: actors)
                m_broadphase.update_proxy(data.proxy_id, data.actor->shape().aabb());

        m_broadphase.find_overlaps_pass0();
    }, "prepare_broadphase");

    executor.set_order(pre_fence, prepare_broadphase);

    TaskId merge_overlaps = executor.emplace([this](){
        m_overlaps.clear();
        for (auto& ctx : m_broadphase_ctx) {
            m_overlaps.insert(m_overlaps.end(), ctx.overlaps.begin(), ctx.overlaps.end());
            ctx.overlaps.clear();
        }
    }, "merge_overlaps");

    for (int worker_id = 0; worker_id < bp_concurrency; worker_id++) {
        TaskId task = executor.emplace([this, worker_id](){
            auto& ctx = m_broadphase_ctx[worker_id];
            m_broadphase.find_overlaps_pass1(worker_id, ctx.overlaps);
        }, "broadphase");

        executor.set_order(prepare_broadphase, task, merge_overlaps);
    }

    TaskId merge_contacts_task = executor.emplace([this]() {
        merge_contacts();
    }, "merge_contacts");

    executor.set_order(merge_contacts_task, post_fence);

    for (int worker_id = 0; worker_id < np_concurrency; worker_id++) {
        TaskId task = executor.emplace([this, worker_id, np_concurrency](){
            auto& ctx = m_narrowphase_ctx[worker_id];
            auto [chunk_beg, chunk_end] = select_sequence_chunk(worker_id, np_concurrency, m_overlaps);
            for (auto it = chunk_beg; it != chunk_end; ++it)
                collide(ctx, it->data1, it->data2);
        }, "narrowphase");

        executor.set_order(merge_overlaps, task, merge_contacts_task);
    }
}

void DynamicsWorld::setup_executor(TaskExecutor& executor)
{
    int concurrency = executor.concurrency();
    SL_VERIFY(concurrency > 0);

    m_solve_ctx.resize(concurrency);
    for (auto& ctx : m_solve_ctx)
        ctx.solver.set_concurrency(concurrency);

    TaskId prepare_task = executor.emplace([this]() {
        if (m_debug_drawer)
            m_debug_drawer->clear();

        if (m_np_backend_hint != m_config.np_backend_hint)
            setup_narrowphase();

        setup_solver();

        reset_frame_stats();

        if (m_config.enable_integration && m_config.delay_integration)
            integrate_bodies(0, 1);

    }, "prepare");

    TaskId external_forces_task = executor.emplace([this]() {
        apply_external_forces();
    }, "external_forces");

    TaskId allocate = executor.emplace([this]() { allocate_constraints(); }, "allocate");
    executor.set_order(prepare_task, external_forces_task, allocate);

    setup_collision_detection(executor, prepare_task, allocate);

    /*
    TaskId joints_task = executor.emplace([this]() {
        if (m_config.enable_constraint_resolving)
            apply_joints();
    }, "joint_constraints");
*/

    TaskId after_randomize = executor.emplace([]() {}, "after_randomize");
    for (int solver_id = 0; solver_id < concurrency; solver_id++) {
        TaskId randomize = executor.emplace([this, solver_id]() { randomize_contacts(solver_id); }, "randomize_minor");
        executor.set_order(allocate, randomize, after_randomize);
    }

    TaskId before_pass0 = executor.emplace([]() {}, "before_solver_pass0");
    TaskId after_pass0 = executor.emplace([]() {}, "before_solver_pass0");

    for (int worker_id = 0; worker_id < concurrency; worker_id++) {
        TaskId pre = after_randomize;
        for (int solver_id = 0; solver_id < concurrency; solver_id++) {
            TaskId task = executor.emplace([this, worker_id, concurrency, solver_id]() {
                apply_contacts(worker_id, concurrency, solver_id);
            }, "apply_contacts");
            executor.set_order(pre, task);
            pre = task;
        }

        executor.set_order(pre, before_pass0);
    }

    for (int worker_id = 0; worker_id < concurrency; worker_id++) {
        TaskId task = executor.emplace([this, worker_id]() {
            m_solve_ctx[worker_id].solver.solve_pass0();
        }, "solver_pass0");
        executor.set_order(before_pass0, task, after_pass0);
    }

    TaskId after_pass1 = executor.emplace([]() {}, "after_solver_pass1");

    for (int worker_id = 0; worker_id < concurrency; worker_id++) {
        TaskId pre = after_pass0;
        for (int solver_id = 0; solver_id < concurrency; solver_id++) {
            TaskId task = executor.emplace([this, solver_id, worker_id]() {
                m_solve_ctx[solver_id].solver.solve_pass1(worker_id);
            }, "solver_pass1");
            executor.set_order(pre, task);
            pre = task;
        }

        executor.set_order(pre, after_pass1);
    }

    TaskId after_solver = executor.emplace([]() {}, "after_solver");

    for (int worker_id = 0; worker_id < concurrency; worker_id++) {
        TaskId pass2 = executor.emplace([this, worker_id]() {
            m_solve_ctx[worker_id].solver.solve_pass2();
        }, "solver_pass2");
        executor.set_order(after_pass1, pass2, after_solver);
    }

    TaskId after_cache_lambda = executor.emplace([]() {}, "after_cache_lambda");

    for (int worker_id = 0; worker_id < concurrency; worker_id++) {
        TaskId cache_lambda_task = executor.emplace([this, worker_id, concurrency]() {
            if (m_config.enable_constraint_resolving) {
                cache_lambdas(worker_id, concurrency);
            }
        }, "cache_lambda");

        executor.set_order(after_solver, cache_lambda_task, after_cache_lambda);
    }

    TaskId cleanup = executor.emplace([this]() {
        if (m_config.enable_constraint_resolving) {
            update_constraint_stats();
        }

        update_general_stats();

        for (auto& ctx : m_solve_ctx) {
            ctx.solver.clear();
            ctx.pending_contacts.clear();
        }

        refresh_manifolds();
    }, "cleanup");

    executor.set_order(after_cache_lambda, cleanup);

    int integrate_concurrency = concurrency * 3;
    for (int worker_id = 0; worker_id < integrate_concurrency; worker_id++) {
        TaskId integrate_task = executor.emplace([this, worker_id, integrate_concurrency]() {
            if (m_config.enable_integration && !m_config.delay_integration)
                integrate_bodies(worker_id, integrate_concurrency);
        }, "integrate");
        executor.set_order(after_cache_lambda, integrate_task);
    }
}

} // slope
