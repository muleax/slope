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
    Vec3 axis1;
    Vec3 axis2;
};

std::optional<FrictionBasis> get_velocity_dependent_friction_basis(
    RigidBody* body1, RigidBody* body2, ConstraintGeom& geom, float threshold)
{
    std::optional<FrictionBasis> basis;

    Vec3 vel = body1->point_velocity(geom.p1);
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
    setup_solver(m_config.solver_type, 1.f);
}

void DynamicsWorld::setup_solver(SolverType type, float dt)
{
    if (m_solver_type != type) {
        m_solver_type = type;

        switch (type) {
        case SolverType::PGS:
            m_solver = std::make_unique<ConstraintSolver>();
            break;
        case SolverType::PJ:
            m_solver = std::make_unique<PJConstraintSolver>();
            break;
        }
    }

    m_solver->config() = m_config.solver_config;
    m_solver->set_time_interval(dt);
}

void DynamicsWorld::setup_narrowphase(NpBackendHint hint)
{
    if (m_np_backend_hint != hint) {
        m_np_backend_hint = hint;

        m_narrowphase.reset_all_backends();

        if (hint == NpBackendHint::Mixed) {
            m_narrowphase.add_backend<GJKConvexPolyhedronBackend>();
            m_narrowphase.add_backend<GJKConvexPolyhedronBoxBackend>();
            m_narrowphase.add_backend<SATBoxBackend>();

        } else if (hint == NpBackendHint::GJK_EPA) {
            m_narrowphase.add_backend<GJKConvexPolyhedronBackend>();
            m_narrowphase.add_backend<GJKConvexPolyhedronBoxBackend>();
            m_narrowphase.add_backend<GJKBoxBackend>();

        } else {
            m_narrowphase.add_backend<SATConvexPolyhedronBackend>();
            m_narrowphase.add_backend<SATConvexPolyhedronBoxBackend>();
            m_narrowphase.add_backend<SATBoxBackend>();
        }

        m_narrowphase.add_backend<ConvexPolyhedronCapsuleBackend>();
        m_narrowphase.add_backend<ConvexPolyhedronSphereBackend>();

        m_narrowphase.add_backend<BoxCapsuleBackend>();
        m_narrowphase.add_backend<BoxSphereBackend>();

        m_narrowphase.add_backend<CapsuleSphereBackend>();
        m_narrowphase.add_backend<CapsuleBackend>();

        m_narrowphase.add_backend<SphereBackend>();
    }
}

void DynamicsWorld::add_actor(BaseActor* actor)
{
    auto proxy_id = m_broadphase.add_proxy(actor);

    if (actor->is<DynamicActor>()) {
        m_dynamic_actors.push_back({static_cast<DynamicActor*>(actor), proxy_id});

    } else {
        SL_VERIFY(actor->is<StaticActor>());
        m_static_actors.push_back({static_cast<StaticActor*>(actor), proxy_id});
    }
}

void DynamicsWorld::remove_actor(BaseActor* actor)
{
    // TODO: remove associated joints
    if (actor->is<DynamicActor>()) {
        remove_actor_impl(m_dynamic_actors, actor);

    } else {
        SL_VERIFY(actor->is<StaticActor>());
        remove_actor_impl(m_static_actors, actor);
    }
}

void DynamicsWorld::add_joint(Joint* joint)
{
    m_joints.push_back(joint);
}

void DynamicsWorld::remove_joint(Joint* joint)
{
    auto it = std::remove(m_joints.begin(), m_joints.end(), joint);
    m_joints.erase(it, m_joints.end());
}

void DynamicsWorld::clear()
{
    for (auto& actor: m_dynamic_actors)
        m_broadphase.remove_proxy(actor.proxy_id);

    m_dynamic_actors.clear();

    for (auto& actor: m_static_actors)
        m_broadphase.remove_proxy(actor.proxy_id);

    m_static_actors.clear();

    m_joints.clear();
}

template<class T>
void DynamicsWorld::remove_actor_impl(Vector<T>& container, BaseActor* actor)
{
    auto it = std::find_if(container.begin(), container.end(),
                           [&actor](const auto& data) { return data.actor == actor; });
    if (it == container.end()) {
        log::error("Remove actor: actor not found");
        return;
    }

    m_broadphase.remove_proxy(it->proxy_id);

    container.erase(it);
}

void DynamicsWorld::set_debug_drawer(std::shared_ptr<DebugDrawer> drawer)
{
    m_debug_drawer = std::move(drawer);
}

void DynamicsWorld::apply_gravity()
{
    for (auto& data: m_dynamic_actors) {
        data.actor->body().apply_force_to_com(m_config.gravity * data.actor->body().mass());
    }
}

void DynamicsWorld::apply_gyroscopic_torque(float dt)
{
    for (auto& data: m_dynamic_actors) {
        data.actor->body().apply_gyroscopic_torque(dt);
    }
}

void DynamicsWorld::collide(BaseActor* actor1, BaseActor* actor2)
{
    if (actor1 > actor2)
        std::swap(actor1, actor2);

    if (!actor1->is<DynamicActor>())
        std::swap(actor1, actor2);

    auto& shape1 = actor1->shape();
    auto& shape2 = actor2->shape();

    m_stats.np_test_count++;

    m_contact_patch.reset();
    if (m_narrowphase.collide(&shape1, &shape2, m_contact_patch)) {
        m_stats.collision_count++;

        auto& cache = m_manifolds[{&shape1, &shape2}];
        // TODO: check in broadphase
        cache.actor1 = static_cast<DynamicActor*>(actor1);
        cache.actor2 = actor2;
        cache.touch_frame_id = m_frame_id;

        auto& manifold = cache.manifold;
        manifold.update_inv_transform(actor1->inv_transform());
        manifold.begin_update();

        m_contact_patch.normalize_order();
        for (auto& geom: m_contact_patch.contacts)
            manifold.add_contact(geom);

        manifold.end_update();

        for (auto& p: manifold)
            m_pending_contacts.push_back({&cache, &p});
    }
}

void DynamicsWorld::perform_collision_detection()
{
    m_stats.np_test_count = 0;
    m_stats.collision_count = 0;
    m_narrowphase.gjk_solver().reset_stats();
    m_narrowphase.epa_solver().reset_stats();
    m_narrowphase.sat_solver().reset_stats();

    for (auto& data: m_dynamic_actors)
        m_broadphase.update_proxy(data.proxy_id, data.actor->shape().aabb());

    for (auto& data: m_static_actors)
        m_broadphase.update_proxy(data.proxy_id, data.actor->shape().aabb());

    m_broadphase.find_overlapping_pairs([this](BaseActor* actor1, BaseActor* actor2) {
        collide(actor1, actor2);
    });
}

void DynamicsWorld::apply_contacts()
{
    m_stats.contact_count = m_pending_contacts.size();

    auto* debug_drawer = m_debug_drawer.get();

    if (m_config.randomize_order) {
        for (auto& c: m_pending_contacts) {
            auto r = rand() % m_pending_contacts.size();
            std::swap(c, m_pending_contacts[r]);
        }
    }

    for (auto[cache, p]: m_pending_contacts) {
        ConstraintGeom geom = {p->geom.p1, p->geom.p2, p->geom.normal};

        auto* actor1 = cache->actor1;
        auto* actor2 = cache->actor2;

        SL_ASSERT(actor1->is<DynamicActor>());
        RigidBody* body1 = &actor1->body();
        RigidBody* body2 = nullptr;
        if (actor2->is<DynamicActor>())
            body2 = &static_cast<DynamicActor*>(actor2)->body();

        Constraint nc = Constraint::stabilized_unilateral(body1, body2, geom);

        std::optional<FrictionBasis> basis;
        if (m_config.enable_velocity_dependent_friction) {
            // TODO: reconsider threshold
            basis = get_velocity_dependent_friction_basis(body1, body2, geom, 0.3f);
        }

        if (!basis) {
            basis.emplace();
            find_tangent(basis->axis1, basis->axis2, geom.axis);
        }

        auto fc1 = Constraint::bilateral(body1, body2, {geom.p1, geom.p2, basis->axis1});
        auto fc2 = Constraint::bilateral(body1, body2, {geom.p1, geom.p2, basis->axis2});

        // TODO: implement policies
        float friction_ratio = actor1->friction() * actor2->friction();

        nc.init_lambda = p->normal_lambda * m_config.warmstarting_normal;
        fc1.init_lambda = p->friction1_lambda * m_config.warmstarting_friction;
        fc2.init_lambda = p->friction2_lambda * m_config.warmstarting_friction;

        p->normal_constr_id = m_solver->add_constraint(nc);

        if (m_config.enable_cone_friction) {
            auto ids = m_solver->join_friction_cone(fc1, fc2, {friction_ratio, friction_ratio}, p->normal_constr_id);
            p->friction1_constr_id = ids.first;
            p->friction2_constr_id = ids.second;

        } else {
            //p->friction1_constr_id = m_solver->join_friction(fc1, friction_ratio, p->normal_constr_id);
            //p->friction2_constr_id = m_solver->join_friction(fc2, friction_ratio, p->normal_constr_id);
            auto ids = m_solver->join_friction_2d(fc1, fc2, {friction_ratio, friction_ratio}, p->normal_constr_id);
            p->friction1_constr_id = ids.first;
            p->friction2_constr_id = ids.second;
        }

        if (debug_drawer) {
            if (m_config.draw_contact_normals1) {
                debug_drawer->draw_line(geom.p1, geom.p1 + geom.axis * 0.3f, {0.2f, 1.f, 0.2f});
            }

            if (m_config.draw_contact_friction1) {
                debug_drawer->draw_line(geom.p1, geom.p1 + fc1.jacobian1[0] * 0.3f, {1.f, 0.1f, 0.2f});
                debug_drawer->draw_line(geom.p1, geom.p1 + fc2.jacobian1[0] * 0.3f, {1.f, 0.1f, 0.2f});
            }

            if (m_config.draw_contact_normals2) {
                debug_drawer->draw_line(geom.p2, geom.p2 - geom.axis * 0.3f, {0.2f, 1.f, 0.2f});
            }

            if (m_config.draw_contact_friction2) {
                debug_drawer->draw_line(geom.p2, geom.p2 + fc1.jacobian1[0] * 0.3f, {1.f, 0.1f, 0.2f});
                debug_drawer->draw_line(geom.p2, geom.p2 + fc2.jacobian1[0] * 0.3f, {1.f, 0.1f, 0.2f});
            }
        }
    }
}

void DynamicsWorld::apply_joints()
{
    for (auto* joint : m_joints) {
        joint->set_warmstarting_ratio(m_config.warmstarting_joint);
        joint->apply_constraints(m_solver.get());
    }
}

void DynamicsWorld::cache_lambdas()
{
    for (auto[_, p]: m_pending_contacts) {
        p->normal_lambda = m_solver->get_lambda(p->normal_constr_id);
        p->friction1_lambda = m_solver->get_lambda(p->friction1_constr_id);
        p->friction2_lambda = m_solver->get_lambda(p->friction2_constr_id);
    }

    for (auto* joint : m_joints) {
        joint->cache_lambdas(m_solver.get());
    }
}

void DynamicsWorld::integrate_bodies()
{
    float dt = m_solver->time_interval();

    for (auto& data: m_dynamic_actors) {
        auto& body = data.actor->body();
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
}

void DynamicsWorld::update_constraint_stats()
{
}

void DynamicsWorld::update_general_stats()
{
    m_stats.static_actor_count = m_static_actors.size();
    m_stats.dynamic_actor_count = m_dynamic_actors.size();
    m_stats.simulation_time += m_solver->time_interval();
}

void DynamicsWorld::update(float dt)
{
    if (m_debug_drawer)
        m_debug_drawer->clear();

    if (m_config.enable_integration && m_config.delay_integration)
        integrate_bodies();

    setup_narrowphase(m_config.np_backend_hint);
    setup_solver(m_config.solver_type, dt);

    if (m_config.enable_gravity)
        apply_gravity();

    if (m_config.enable_gyroscopic_torque)
        apply_gyroscopic_torque(dt);

    perform_collision_detection();

    apply_contacts();

    apply_joints();

    if (m_config.enable_constraint_resolving) {
        m_solver->solve();
        cache_lambdas();
        update_constraint_stats();
    }

    m_solver->clear();
    m_pending_contacts.clear();

    refresh_manifolds();

    if (m_config.enable_integration && !m_config.delay_integration)
        integrate_bodies();

    update_general_stats();

    m_frame_id++;
}

} // slope
