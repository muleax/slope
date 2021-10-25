#include "app/ecs/world.hpp"
#include "app/ecs/system.hpp"

namespace slope::app {

World::World() {
    auto& registry = ComponentRegistry::instance();

    for (ComponentType::Id type_id = 0; type_id < registry.limit_id(); type_id++) {
        auto& type = registry.get_type(type_id);
        m_components[type_id].emplace(&type);
    }

    m_singleton_entity = create_entity();
}

World::~World() {
    auto limit_id = static_cast<Entity::Id>(m_entities.size());
    for (Entity::Id entity_id = 0; entity_id < limit_id; entity_id++) {
        destroy_entity(Entity{entity_id});
    }
}

Entity World::create_entity() {
    // TODO: concurrency issues
    // std::lock_guard<std::mutex> lock(m_entity_mutex);

    Entity::Id free_id = Entity::INVALID_ID;
    if (!m_free_entity_ids.empty()) {
        free_id = m_free_entity_ids.back();
        m_free_entity_ids.pop_back();
    } else {
        free_id = static_cast<Entity::Id>(m_entities.size());
        m_entities.emplace_back();
    }

    m_entities[free_id].alive = true;

    return Entity(free_id);
}

void World::destroy_entity(Entity e) {
    // TODO: concurrency issues
    // std::lock_guard<std::mutex> lock(m_entity_mutex);
    m_entity_destroy_orders.push_back(e);
}

void World::process_commands() {
    // TODO: optimize

    for (auto [e, type_id] : m_component_destroy_orders) {
        auto& cdata = *m_components[type_id];
        auto& edata = m_entities[e.id()];

        cdata.manager.destroy_component(e);

        for (auto view_id : cdata.related_views) {
            auto& view = *m_views[view_id];
            if (edata.mask.is_match(view.filter())) {
                view.remove(e);
            }
        }

        edata.mask.reset(type_id);
    }

    for (auto e : m_entity_destroy_orders) {
        auto& edata = m_entities[e.id()];
        auto limit_id = ComponentRegistry::instance().limit_id();

        for (ComponentType::Id type_id = 0; type_id < limit_id; type_id++) {
            if (edata.mask.test(type_id)) {
                m_components[type_id]->manager.destroy_component(e);
            }
        }

        for (auto* view : m_active_views) {
            if (edata.mask.is_match(view->filter())) {
                view->remove(e);
            }
        }

        edata.mask.reset();
        edata.alive = false;

        m_free_entity_ids.push_back(e.id());
    }

    for (auto [e, type_id] : m_component_create_orders) {
        auto& cdata = *m_components[type_id];
        auto& edata = m_entities[e.id()];

        edata.mask.set(type_id);

        for (auto view_id : cdata.related_views) {
            auto& view = *m_views[view_id];
            if (edata.mask.is_match(view.filter())) {
                view.add(e);
            }
        }
    }

}

void World::update(float dt) {
    process_commands();

    for (auto& system : m_systems) {
        system->update(dt);
    }
}

} // slope::app