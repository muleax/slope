#pragma once
#include "app/ecs/entity.hpp"
#include "app/ecs/entity_view.hpp"
#include "app/ecs/component.hpp"
#include "slope/core/vector.hpp"
#include "slope/core/array.hpp"
#include <optional>
#include <mutex>

namespace slope::app {

class System;

class World {
public:
    World();
    ~World();

    Entity create_entity();
    void destroy_entity(Entity e);

    template <class T>
    T* create(Entity e) {
        auto type_id = T::type_id();

        // TODO: concurrency issues
        m_component_create_orders.push_back({e, type_id});

        auto& cdata = *m_components[type_id];
        return static_cast<T*>(cdata.manager.create_component(e));
    }

    template <class T>
    void destroy(Entity e) {
        auto type_id = T::type_id();
        // TODO: concurrency issues
        m_component_destroy_orders.push_back({e, type_id});
    }

    template <class T>
    const T* get(Entity e) const {
        return static_cast<const T*>(m_components[T::type_id()]->manager.get_component(e));
    }

    template <class T>
    T* modify(Entity e) {
        return static_cast<T*>(m_components[T::type_id()]->manager.get_component(e));
    }

    template <class T>
    bool has(Entity e) {
        return m_entities[e.id()].mask.test(T::type_id());
    }

    template <class T>
    T* create_singleton() {
        return create<T>(m_singleton_entity);
    }

    template <class T>
    const T* get_singleton() const {
        return get<T>(m_singleton_entity);
    }

    template <class T>
    T* modify_singleton() {
        return modify<T>(m_singleton_entity);
    }

    const ComponentMask& get_component_mask(Entity e) const { return m_entities[e.id()].mask; }

    template <class T>
    void add_system() {
        static_assert(std::is_base_of_v<System, T>);
        m_systems.emplace_back(new T(this));
    }

    // TODO: init/fini views
    template <class... Ts>
    const EntityView& view() {
        auto view_id = EntityViewRegistry::instance().get_id<Ts...>();
        auto& view = m_views[view_id];
        if (!view) {

            // TODO: concurrency
            auto filter = ComponentMask::pack<Ts...>();
            view.emplace(filter);

            m_active_views.push_back(&*view);

            for (Entity::Id entity_id = 0; entity_id < m_entities.size(); ++entity_id) {
                auto& data = m_entities[entity_id];
                if (data.alive && data.mask.is_match(filter)) {
                    view->add(Entity(entity_id));
                }
            }

            (add_related_view<Ts>(view_id), ...);
        }

        return *view;
    }

    template<class T>
    void visit_entities(T&& visitor) {
        for (Entity::Id eid = 1; eid < m_entities.size(); eid++) {
            if (m_entities[eid].alive)
                visitor(Entity{eid});
        }
    }

    void update(float dt);

private:
    struct EntityData {
        ComponentMask mask;
        bool alive = false;
    };

    struct ComponentData {
        explicit ComponentData(const ComponentType* type) : manager(type) {}

        ComponentManager manager;
        Vector<EntityView::Id> related_views;
        //std::mutex m_mutex;
    };

    template<class T>
    void add_related_view(EntityView::Id view_id) {
        auto& cdata = *m_components[T::type_id()];
        //std::lock_guard<std::mutex> guard(cdata.mutex);
        cdata.related_views.push_back(view_id);
    }

    void process_commands();

    Vector<Entity::Id> m_free_entity_ids;
    Array<std::optional<ComponentData>, ComponentType::LIMIT_ID> m_components;
    Array<std::optional<EntityView>, EntityView::LIMIT_ID> m_views;
    Vector<EntityView*> m_active_views;
    Vector<EntityData> m_entities;
    Vector<std::unique_ptr<System>> m_systems;

    struct ComponentRoute {
        Entity entity;
        ComponentType::Id type_id;
    };

    Vector<ComponentRoute> m_component_destroy_orders;
    Vector<ComponentRoute> m_component_create_orders;

    Vector<Entity> m_entity_destroy_orders;

    Entity m_singleton_entity;

    // std::mutex m_entity_mutex;
};

} // slope::app
