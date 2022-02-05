#include "slope/dynamics/dynamics_world.hpp"
#include "slope/dynamics/constraint_solver.hpp"
#include "slope/dynamics/pj_constraint_solver.hpp"
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
    if (init_config)
        m_config = *init_config;

    setup_narrowphase(m_config.np_backend_hint);
    setup_solver(m_config.solver_type);
}

void DynamicsWorld::setup_solver(SolverType type)
{
    if (m_solver_type != type) {
        m_solver_type = type;

        switch (type) {
        case SolverType::PGS:
            m_solver = std::make_unique<ConstraintSolver>();
            break;
        case SolverType::PJ:
            //m_solver = std::make_unique<PJConstraintSolver>();
            break;
        }
    }

    m_solver->config() = m_config.solver_config;
    m_solver->set_time_interval(m_config.time_interval);
}

void DynamicsWorld::setup_narrowphase(NpBackendHint hint)
{
    if (m_np_backend_hint != hint) {
        m_np_backend_hint = hint;

        for (auto& ctx : m_worker_ctx) {
            auto& narrowphase = ctx.narrowphase;
            narrowphase.reset_all_backends();

            if (hint == NpBackendHint::Mixed) {
                narrowphase.add_backend<GJKConvexPolyhedronBackend>();
                narrowphase.add_backend<GJKConvexPolyhedronBoxBackend>();
                narrowphase.add_backend<SATBoxBackend>();

            } else if (hint == NpBackendHint::GJK_EPA) {
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
    for (auto& actors : m_actors) {
        for (auto& data: actors) {
            m_broadphase.remove_proxy(data.proxy_id);
        }
        actors.clear();
    }

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
        m_solver->register_body(&actor->body());
    }
}

void DynamicsWorld::collide(BaseActor* actor1, BaseActor* actor2, WorkerContext& ctx)
{
    if (actor1 > actor2)
        std::swap(actor1, actor2);

    if (!actor1->is<DynamicActor>()) {
        if (actor2->is<DynamicActor>())
            std::swap(actor1, actor2);
        else
            // ignore static to static collisions
            // TODO: filter out this case earlier in broadphase
            return;
    }

    auto& shape1 = actor1->shape();
    auto& shape2 = actor2->shape();

    //m_stats.np_test_count++;

    ctx.contact_patch.reset();

    if (ctx.narrowphase.collide(&shape1, &shape2, ctx.contact_patch)) {
        //m_stats.collision_count++;

        ManifoldCache* cache = nullptr;

        auto it = m_manifolds.find({&shape1, &shape2});
        if (it != m_manifolds.end()) {
            cache = &it->second;
        } else {
            cache = &ctx.new_manifolds[{&shape1, &shape2}];
        }

        // TODO: check in broadphase
        cache->actor1 = static_cast<DynamicActor*>(actor1);
        cache->actor2 = actor2;
        cache->touch_frame_id = m_frame_id;

        auto& manifold = cache->manifold;
        manifold.update_inv_transform(actor1->inv_transform());
        manifold.begin_update();

        ctx.contact_patch.normalize_order();
        for (auto& geom : ctx.contact_patch.contacts)
            manifold.add_contact(geom);

        manifold.end_update();

        auto& pending_contacts = ctx.pending_contacts;
        for (auto& p : manifold) {
            // TODO: implement policies
            float friction_ratio = actor1->friction() * actor2->friction();

            //pending_contacts.push_back({cache, &p, friction_ratio});
            auto& pc = pending_contacts.emplace_back();
            pc.mf_cache = cache;
            pc.mf_point = &p;

            ConstraintGeom geom = {p.geom.p1, p.geom.p2, p.geom.normal};

            SL_ASSERT(actor1->is<DynamicActor>());
            RigidBody* body1 = &actor1->cast<DynamicActor>()->body();
            RigidBody* body2 = actor2->is<DynamicActor>() ? &actor2->cast<DynamicActor>()->body() : nullptr;

            pc.nc = Constraint::stabilized_unilateral(body1, body2, geom);
            pc.nc.init_lambda = p.normal_lambda * m_config.warmstarting_normal;

            std::optional<FrictionBasis> basis;
            if (m_config.enable_velocity_dependent_friction) {
                // TODO: reconsider threshold
                basis = get_velocity_dependent_friction_basis(body1, body2, geom, 0.3f);
            }

            if (!basis) {
                basis.emplace();
                find_tangent(basis->axis1, basis->axis2, geom.axis);
            }
            //pc.friction_u = basis->axis1;
            //pc.friction_v = basis->axis2;


            pc.fc1 = Constraint::bilateral(body1, body2, {geom.p1, geom.p2, basis->axis1});
            pc.fc2 = Constraint::bilateral(body1, body2, {geom.p1, geom.p2, basis->axis2});

            pc.fc1.init_lambda = p.friction1_lambda * m_config.warmstarting_friction;
            pc.fc2.init_lambda = p.friction2_lambda * m_config.warmstarting_friction;

            pc.friction_ratio = friction_ratio;
        }
    }
}

void DynamicsWorld::setup_collision_detection(TaskExecutor& executor, Fence fence)
{
    TaskId prepare_task = executor.emplace([this](){
        for (auto& actors : m_actors)
            for (auto& data : actors)
                m_broadphase.update_proxy(data.proxy_id, data.actor->shape().aabb());
    }, "update_proxies");

    executor.set_order(fence.pre, prepare_task);

    int cd_concurrency = (executor.concurrency() > 1)
        ? executor.concurrency() * 4
        : 1;

    auto overlap_callback = [this](BaseActor* actor1, BaseActor* actor2, int task_index) {
        this->collide(actor1, actor2, m_worker_ctx[task_index]);
    };

    m_broadphase.setup_executor(overlap_callback, executor, {prepare_task, fence.post}, cd_concurrency);
}

void DynamicsWorld::apply_friction()
{
    int nid = 0;
    for (auto& ctx : m_worker_ctx) {
        for (auto& pc : ctx.pending_contacts) {
            auto& p = pc.mf_point;

            //auto& nc = pc.nc;
            auto& fc1 = pc.fc1;
            auto& fc2 = pc.fc2;

/*
            ConstraintGeom geom = {p->geom.p1, p->geom.p2, p->geom.normal};

            auto* actor1 = pc.mf_cache->actor1;
            auto* actor2 = pc.mf_cache->actor2;

            SL_ASSERT(actor1->is<DynamicActor>());
            RigidBody* body1 = &actor1->cast<DynamicActor>()->body();
            RigidBody* body2 = actor2->is<DynamicActor>() ? &actor2->cast<DynamicActor>()->body() : nullptr;

            auto nc = Constraint::stabilized_unilateral(body1, body2, geom);
            nc.init_lambda = p->normal_lambda * m_config.warmstarting_normal;
            m_solver->add_constraint(nc);

            auto fc1 = Constraint::bilateral(body1, body2, {geom.p1, geom.p2, pc.friction_u});
            auto fc2 = Constraint::bilateral(body1, body2, {geom.p1, geom.p2, pc.friction_v});

            fc1.init_lambda = p->friction1_lambda * m_config.warmstarting_friction;
            fc2.init_lambda = p->friction2_lambda * m_config.warmstarting_friction;
*/

            p->normal_constr_id = ConstraintId(ConstraintGroup::General, nid);
            nid++;

            if (m_config.enable_cone_friction) {
                auto ids = m_solver->join_friction_cone(fc1, fc2, {pc.friction_ratio, pc.friction_ratio},
                                                        p->normal_constr_id);
                p->friction1_constr_id = ids.first;
                p->friction2_constr_id = ids.second;

            } else {
                //p->friction1_constr_id = m_solver->join_friction(fc1, friction_ratio, p->normal_constr_id);
                //p->friction2_constr_id = m_solver->join_friction(fc2, friction_ratio, p->normal_constr_id);
                auto ids = m_solver->join_friction_2d(fc1, fc2, {pc.friction_ratio, pc.friction_ratio}, p->normal_constr_id);
                p->friction1_constr_id = ids.first;
                p->friction2_constr_id = ids.second;
            }
        }
    }

}

void DynamicsWorld::apply_contacts()
{
    //m_stats.contact_count = m_pending_contacts.size();

    auto* debug_drawer = m_debug_drawer.get();

    /*
    if (m_config.randomize_order) {
        for (auto& c: m_pending_contacts) {
            auto r = rand() % m_pending_contacts.size();
            std::swap(c, m_pending_contacts[r]);
        }
    }
     */

    for (auto& ctx : m_worker_ctx) {
        for (auto& pc : ctx.pending_contacts) {
            auto& p = pc.mf_point;

            auto& nc = pc.nc;
            //auto& fc1 = pc.fc1;
            //auto& fc2 = pc.fc2;

            /*
            ConstraintGeom geom = {p->geom.p1, p->geom.p2, p->geom.normal};

            auto* actor1 = pc.mf_cache->actor1;
            auto* actor2 = pc.mf_cache->actor2;

            SL_ASSERT(actor1->is<DynamicActor>());
            RigidBody* body1 = &actor1->cast<DynamicActor>()->body();
            RigidBody* body2 = actor2->is<DynamicActor>() ? &actor2->cast<DynamicActor>()->body() : nullptr;
*/


            //p->normal_constr_id = m_solver->add_constraint(nc);
            //auto nc = Constraint::stabilized_unilateral(body1, body2, geom);
            //nc.init_lambda = p->normal_lambda * m_config.warmstarting_normal;

            m_solver->add_constraint(nc);

            /*
            if (m_config.enable_cone_friction) {
                auto ids = m_solver->join_friction_cone(fc1, fc2, {pc.friction_ratio, pc.friction_ratio},
                                                        p->normal_constr_id);
                p->friction1_constr_id = ids.first;
                p->friction2_constr_id = ids.second;

            } else {
                //p->friction1_constr_id = m_solver->join_friction(fc1, friction_ratio, p->normal_constr_id);
                //p->friction2_constr_id = m_solver->join_friction(fc2, friction_ratio, p->normal_constr_id);
                auto ids = m_solver->join_friction_2d(fc1, fc2, {pc.friction_ratio, pc.friction_ratio}, p->normal_constr_id);
                p->friction1_constr_id = ids.first;
                p->friction2_constr_id = ids.second;
            }
*/
/*
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
    }
}

void DynamicsWorld::apply_joints()
{
    for (auto& joint : m_joints) {
        joint->set_warmstarting_ratio(m_config.warmstarting_joint);
        joint->apply_constraints(m_solver.get());
    }
}

void DynamicsWorld::cache_lambdas()
{
    for (auto& ctx : m_worker_ctx) {
        for (auto& pc : ctx.pending_contacts) {
            auto* p = pc.mf_point;
            p->normal_lambda = m_solver->get_lambda(p->normal_constr_id);
            p->friction1_lambda = m_solver->get_lambda(p->friction1_constr_id);
            p->friction2_lambda = m_solver->get_lambda(p->friction2_constr_id);
        }
    }

    for (auto& joint : m_joints) {
        joint->cache_lambdas(m_solver.get());
    }
}

void DynamicsWorld::integrate_bodies()
{
    float dt = m_solver->time_interval();

    for (auto& data : m_actors[(int)ActorKind::Dynamic]) {
        auto* actor = data.actor->cast<DynamicActor>();
        auto& body = actor->body();
        body.integrate(dt);
        data.actor->shape().set_transform(body.transform());
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

    for (auto& ctx : m_worker_ctx) {
        for (auto& [k, mf] : ctx.new_manifolds)
            m_manifolds[k] = mf;

        ctx.new_manifolds.clear();
    }
}

void DynamicsWorld::reset_frame_stats()
{
    m_stats.np_test_count = 0;
    m_stats.collision_count = 0;

    for (auto& ctx : m_worker_ctx) {
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

    m_stats.simulation_time += m_solver->time_interval();
}

void DynamicsWorld::setup_executor(TaskExecutor& executor)
{
    TaskId prepare_task = executor.emplace([this]() {
        if (m_debug_drawer)
            m_debug_drawer->clear();

        if (m_config.enable_integration && m_config.delay_integration)
            integrate_bodies();

        setup_narrowphase(m_config.np_backend_hint);
        setup_solver(m_config.solver_type);

        reset_frame_stats();
    }, "prepare");

    TaskId external_forces_task = executor.emplace([this]() {
        apply_external_forces();
    }, "external_forces");

    TaskId after_collision = executor.emplace([]() {});

    executor.set_order(prepare_task, external_forces_task, after_collision);

    setup_collision_detection(executor, {prepare_task, after_collision});

    TaskId normal_constr_task = executor.emplace([this]() {
        if (m_config.enable_constraint_resolving)
            apply_contacts();
    }, "normal_constraints");

    TaskId friction_constr_task = executor.emplace([this]() {
        if (m_config.enable_constraint_resolving)
            apply_friction();
    }, "friction_constraints");

    TaskId joints_task = executor.emplace([this]() {
        if (m_config.enable_constraint_resolving)
            apply_joints();
    }, "joint_constraints");

    TaskId before_solver = executor.emplace([]() {});

    executor.set_order(after_collision, friction_constr_task, before_solver);
    executor.set_order(after_collision, normal_constr_task, joints_task, before_solver);

    TaskId cache_lambda_task = executor.emplace([this]() {
        if (m_config.enable_constraint_resolving) {
            cache_lambdas();
            update_constraint_stats();
        }

        m_solver->clear();

        for (auto& ctx: m_worker_ctx)
            ctx.pending_contacts.clear();
    }, "cache_lambda");

    m_solver->setup_solve_executor(executor, {before_solver, cache_lambda_task});

    TaskId refresh_manifolds_task = executor.emplace([this]() {
        refresh_manifolds();
    }, "refresh_manifolds");

    executor.set_order(cache_lambda_task, refresh_manifolds_task);

    TaskId integrate_task = executor.emplace([this]() {
        if (m_config.enable_integration && !m_config.delay_integration)
            integrate_bodies();

        update_general_stats();

        m_frame_id++;
    }, "integrate");

    executor.set_order(refresh_manifolds_task, integrate_task);
}

} // slope
