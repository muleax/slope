#include "slope/dynamics/dynamics_world.hpp"
#include "slope/dynamics/pgs_constraint_solver.hpp"
#include "slope/dynamics/pj_constraint_solver.hpp"
#include "slope/debug/log.hpp"
#include <tuple>

namespace slope {

namespace {

// TODO: reconsider
std::pair<Constraint, Constraint> default_contact_friction(RigidBody* body1, RigidBody* body2, ConstraintGeom& geom) {
    Vec3 vel = body1->point_velocity(geom.p1);

    if (body2)
        vel -= body2->point_velocity(geom.p2);

    Vec3 axis1;
    Vec3 axis2;

    bool fallback = true;
    auto lat_vel = vel - geom.axis * geom.axis.dot(vel);
    if (lat_vel.length_squared() > 0.1f) {
        axis1 = lat_vel.normalized();
        axis2 = geom.axis.cross(axis1);
        fallback = false;
    }

    if (fallback)
        find_tangent(axis1, axis2, geom.axis);

    return {
        Constraint::bilateral(body1, body2, {geom.p1, geom.p2, axis1}),
        Constraint::bilateral(body1, body2, {geom.p1, geom.p2, axis2}) };
}

} // unnamed

DynamicsWorld::DynamicsWorld() {
    set_solver(m_solver_type);
}

void DynamicsWorld::set_solver(DynamicsWorld::SolverType type) {
    if (!m_solver || type != m_solver_type) {
        m_solver_type = type;
        switch (type) {
        case SolverType::PGS:
            m_solver = std::make_unique<PGSConstraintSolver>();
            break;
        case SolverType::PJ:
            m_solver = std::make_unique<PJConstraintSolver>();
            break;
        }
    }
}

void DynamicsWorld::add_actor(BaseActor* actor) {
    if (actor->is<DynamicActor>()) {
        m_dynamic_actors.push_back(static_cast<DynamicActor*>(actor));
    } else {
        SL_VERIFY(actor->is<StaticActor>());
        m_static_actors.push_back(static_cast<StaticActor*>(actor));
    }
}

void DynamicsWorld::remove_actor(BaseActor* actor) {
    if (actor->is<DynamicActor>()) {
        remove_actor_impl(m_dynamic_actors, actor);
    } else {
        SL_VERIFY(actor->is<StaticActor>());
        remove_actor_impl(m_static_actors, actor);
    }
}

template<class T>
void DynamicsWorld::remove_actor_impl(Vector<T>& container, BaseActor* actor) {
    auto it = std::find(container.begin(), container.end(), actor);
    if (it == container.end()) {
        log::error("Remove actor: actor not found");
        return;
    }

    container.erase(it);
}

void DynamicsWorld::set_debug_drawer(std::shared_ptr<DebugDrawer> drawer) {
    m_debug_drawer = std::move(drawer);
}

void DynamicsWorld::apply_gravity() {
    for (auto& actor: m_dynamic_actors) {
        actor->body().apply_force_to_com(m_config.gravity * actor->body().mass());
    }
}

void DynamicsWorld::collide(BaseActor& actor1, BaseActor& actor2) {
    auto& shape1 = actor1.shape();
    auto& shape2 = actor2.shape();

    if (!shape1.aabb().intersects(shape2.aabb()))
        return;

    //auto pen_axis = m_narrowphase.find_penetration_axis(&shape1, &shape2);
    if (m_narrowphase.intersect(&shape1, &shape2)) {
        m_stats.collision_count++;

        auto& cache = m_manifolds[{ &shape1, &shape2 }];
        cache.actor1 = &actor1;
        cache.actor2 = &actor2;

        auto& manifold = cache.manifold;
        manifold.begin_update(m_frame_id, actor1.inv_transform());

        //m_narrowphase.generate_contacts(manifold, *pen_axis, &shape1, &shape2);
        m_narrowphase.generate_contacts(manifold);

        manifold.end_update();

        for (auto& p: manifold)
            m_pending_contacts.push_back({&cache, &p});
    }
}

void DynamicsWorld::perform_collision_detection() {
    m_stats.collision_count = 0;

    for (auto actor1 = m_dynamic_actors.begin(); actor1 != m_dynamic_actors.end(); ++actor1)
        for (auto actor2 = actor1 + 1; actor2 != m_dynamic_actors.end(); ++actor2)
            collide(**actor1, **actor2);

    for (auto& actor : m_dynamic_actors)
        for (auto& static_actor : m_static_actors)
            collide(*actor, *static_actor);
}

void DynamicsWorld::apply_contacts() {
    m_stats.contact_count = m_pending_contacts.size();

    auto* debug_drawer = m_debug_drawer.get();

    if (m_config.randomize_order) {
        for (auto& c: m_pending_contacts) {
            auto r = rand() % m_pending_contacts.size();
            std::swap(c, m_pending_contacts[r]);
        }
    }

    for (auto [cache, p]: m_pending_contacts) {
        ConstraintGeom geom = {p->geom.p1, p->geom.p2, p->geom.normal};

        auto* actor1 = cache->actor1;
        auto* actor2 = cache->actor2;

        SL_ASSERT(actor1->is<DynamicActor>());
        RigidBody* body1 = &static_cast<DynamicActor*>(actor1)->body();
        RigidBody* body2 = nullptr;
        if (actor2->is<DynamicActor>())
            body2 = &static_cast<DynamicActor*>(actor2)->body();

        Constraint nc = Constraint::stabilized_unilateral(body1, body2, geom);
        auto [fc1, fc2] = default_contact_friction(body1, body2, geom);

        // TODO: implement policies
        float friction_ratio = actor1->friction() * actor2->friction();

        nc.init_lambda  = p->normal_lambda * m_config.warmstarting_normal;
        fc1.init_lambda = p->friction1_lambda * m_config.warmstarting_friction;
        fc2.init_lambda = p->friction2_lambda * m_config.warmstarting_friction;

        p->normal_constr_id    = m_solver->add_constraint(nc);
        p->friction1_constr_id = m_solver->join_friction(fc1, friction_ratio, p->normal_constr_id);
        p->friction2_constr_id = m_solver->join_friction(fc2, friction_ratio, p->normal_constr_id);

        if (debug_drawer) {
            if (m_config.draw_contact_normals) {
                debug_drawer->draw_line(geom.p1, geom.p1 + geom.axis * 0.3f, {0.2f, 1.f, 0.2f});
            }

            if (m_config.draw_contact_friction) {
                debug_drawer->draw_line(geom.p1, geom.p1 + fc1.jacobian1[0] * 0.3f, {1.f, 0.1f, 0.2f});
                debug_drawer->draw_line(geom.p1, geom.p1 + fc2.jacobian1[0] * 0.3f, {1.f, 0.1f, 0.2f});
            }
        }
    }
}

void DynamicsWorld::cache_lambdas() {
    for (auto [_, p] : m_pending_contacts) {
        p->normal_lambda    = m_solver->get_lambda(p->normal_constr_id);
        p->friction1_lambda = m_solver->get_lambda(p->friction1_constr_id);
        p->friction2_lambda = m_solver->get_lambda(p->friction2_constr_id);
    }

    m_pending_contacts.clear();
}

void DynamicsWorld::integrate_bodies() {
    float dt = m_solver->time_interval();

    for (auto& actor: m_dynamic_actors) {
        auto& body = actor->body();
        body.integrate(dt);
        actor->shape().set_transform(body.transform());
    }
}

void DynamicsWorld::refresh_manifolds() {
    for (auto it = m_manifolds.begin(); it != m_manifolds.end();) {
        if (it->second.manifold.is_active(m_frame_id))
            ++it;
        else
            it = m_manifolds.erase(it);
    }
}

void DynamicsWorld::update(float dt) {
    if (m_debug_drawer)
        m_debug_drawer->clear();

    m_solver->set_time_interval(dt);
    m_solver->config() = m_config.solver_config;

    apply_gravity();

    perform_collision_detection();

    apply_contacts();

    m_solver->solve();

    m_stats.max_constraint_solver_error.update(m_solver->max_error());
    m_stats.avg_constraint_solver_error.update(m_solver->avg_error());
    m_stats.static_actor_count = m_static_actors.size();
    m_stats.dynamic_actor_count = m_dynamic_actors.size();
    m_stats.simulation_time += dt;

    cache_lambdas();

    m_solver->clear();

    integrate_bodies();

    refresh_manifolds();

    m_frame_id++;
}

} // slope
