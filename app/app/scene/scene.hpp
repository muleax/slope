#pragma once
#include "slope/containers/vector.hpp"
#include "slope/containers/unordered_map.hpp"
#include "slope/math/matrix44.hpp"
#include "app/render/model.hpp"
#include <mutex>

namespace slope::app {
/*
class Entity {
public:
    using EntityId = uint32_t;
    static constexpr EntityId INVALID_ID = std::numeric_limits<EntityId>::max();

    explicit Entity(EntityId id) : m_id(id) {}

    EntityId id() const { return m_id; }

private:
    EntityId m_id = INVALID_ID;
};

class Scene {
public:
    Entity          create_entity();
    void            destroy_entity(Entity entity);

    void            set_model(Entity entity, const ModelPtr& model);
    bool            has_model(Entity entity) const;
    const ModelPtr& get_model(Entity entity) const;

    void            set_transform(Entity entity, const Mat44& transform);
    const Mat44&    get_transform(Entity entity) const;

    void            draw();

private:
    using ModelIndex = uint32_t;
    static constexpr ModelIndex INVALID_MODEL_INDEX = std::numeric_limits<ModelIndex>::max();

    struct EntityData {
        ModelIndex model_index = INVALID_MODEL_INDEX;
        Mat44 transform;
    };

    Vector<EntityData> m_entities;
    Vector<Entity::EntityId> m_free_slots;

    Vector<ModelPtr> m_models;
    UnorderedMap<Model::ModelId, uint32_t> m_model_id_to_index;

    std::mutex m_entity_mutex;
    std::mutex m_model_mutex;
};

inline void Scene::set_transform(Entity entity, const Mat44& transform) {
    m_entities[entity.id()].transform = transform;
}

inline const Mat44& Scene::get_transform(Entity entity) const {
    return m_entities[entity.id()].transform;
}
*/
} // slope::app