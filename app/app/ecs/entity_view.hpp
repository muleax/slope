#pragma once
#include "app/ecs/entity.hpp"
#include "app/ecs/component.hpp"
#include "slope/containers/unordered_map.hpp"
#include "slope/containers/unordered_set.hpp"
#include <limits>
#include <mutex>

namespace slope::app {

class EntityViewIterator
{
public:
    using RawIterator = UnorderedSet<Entity>::iterator;
    explicit EntityViewIterator(RawIterator it) : m_it(it) {}

    Entity operator*() const { return *m_it; }
    const Entity* operator->() { return m_it.operator->(); }

    EntityViewIterator& operator++() {
        m_it++;
        return *this;
    }
/*
    EntityViewIterator operator++(int) {
        auto tmp = *this;
        ++m_it;
        return tmp;
    }
*/
    friend bool operator== (const EntityViewIterator& a, const EntityViewIterator& b) {
        return a.m_it == b.m_it;
    }

    friend bool operator!= (const EntityViewIterator& a, const EntityViewIterator& b) {
        return !(a == b);
    };

private:
    RawIterator m_it;
};

class EntityView {
public:
    using Id = uint32_t;
    static constexpr Id LIMIT_ID = 4096;
    static constexpr Id INVALID_ID = std::numeric_limits<Id>::max();

    explicit EntityView(ComponentMask filter) : m_filter(filter) {}

    const ComponentMask& filter() const { return m_filter; }

    EntityViewIterator begin() const { return EntityViewIterator(m_entities.begin()); }
    EntityViewIterator end() const { return EntityViewIterator(m_entities.end()); }

    void add(Entity e) {
        m_entities.insert(e);
    }

    void remove(Entity e) {
        auto it = m_entities.find(e);
        if (it != m_entities.end()) {
            m_entities.erase(it);
        }
    }

private:
    UnorderedSet<Entity> m_entities;
    ComponentMask m_filter;
};

class EntityViewRegistry {
public:
    template<class... Ts>
    EntityView::Id get_id() {
        static std::atomic<EntityView::Id> view_id = EntityView::INVALID_ID;

        if (view_id == EntityView::INVALID_ID) {
            std::lock_guard<std::mutex> lock(m_mutex);

            ComponentMask mask = ComponentMask::pack<Ts...>();
            auto it = m_mask_to_id.find(mask);
            if (it == m_mask_to_id.end()) {
                view_id = static_cast<EntityView::Id>(m_mask_to_id.size());
                SL_VERIFY(view_id < EntityView::LIMIT_ID);
                m_mask_to_id[mask] = view_id;
            } else {
                view_id = it->second;
            }
        }

        return view_id;
    }

    static EntityViewRegistry& instance() { return s_instance; }

private:
    std::mutex m_mutex;
    // TODO: unordered map
    UnorderedMap<ComponentMask, EntityView::Id> m_mask_to_id;

    static EntityViewRegistry s_instance;
};



} // slope::a