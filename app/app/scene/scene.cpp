#include "app/scene/scene.hpp"

namespace slope::app {
/*
Entity Scene::create_entity() {
    std::lock_guard<std::mutex> lock(m_entity_mutex);

    Entity::EntityId allocated_id;
    if (!m_free_slots.empty()) {
        allocated_id = m_free_slots.back();
        m_free_slots.pop_back();
    } else {
        allocated_id = static_cast<Entity::EntityId>(m_entities.size());
        m_entities.emplace_back();
    }

    auto& data = m_entities[allocated_id];
    data.model_index = INVALID_MODEL_INDEX;
    data.transform = Mat44::identity();

    return Entity(allocated_id);
}

void Scene::destroy_entity(Entity entity) {
    std::lock_guard<std::mutex> lock(m_entity_mutex);

    m_free_slots.emplace_back(entity.id());
}

void Scene::set_model(Entity entity, const ModelPtr& model) {
    std::lock_guard<std::mutex> lock(m_model_mutex);

    auto& data = m_entities[entity.id()];

    auto it = m_model_id_to_index.find(model->id());
    if (it != m_model_id_to_index.end()) {
        data.model_index = it->second;
    } else {
        auto model_index = static_cast<ModelIndex>(m_models.size());
        m_models.push_back(model);
        m_model_id_to_index[model->id()] = model_index;
        data.model_index = model_index;
    }
}

bool Scene::has_model(Entity entity) const {
    return m_entities[entity.id()].model_index != INVALID_MODEL_INDEX;
}

const ModelPtr& Scene::get_model(Entity entity) const {
    return m_models[m_entities[entity.id()].model_index];
}

void Scene::draw() {
    
}
*/
} // slope::app