#pragma once
#include "app/ecs/entity.hpp"
#include "slope/containers/string.hpp"
#include "slope/containers/vector.hpp"
#include "slope/containers/array.hpp"
#include "slope/debug/assert.hpp"
#include <atomic>
#include <bitset>

namespace slope::app {

class ComponentType {
public:
    using Id = uint32_t;
    static constexpr Id INVALID_ID = std::numeric_limits<Id>::max();
    static constexpr Id LIMIT_ID = 128;

    Id              id() const { return m_id; }
    const String&   name() const { return m_name; }

    size_t          size() const { return m_size; }
    void            create_component(void* buffer) const { m_constructor(buffer); }
    void            destroy_component(void* buffer) const { m_destructor(buffer); }

private:
    Id                          m_id = INVALID_ID;
    String                      m_name;
    size_t                      m_size;
    std::function<void(void*)>  m_constructor;
    std::function<void(void*)>  m_destructor;

    friend class ComponentRegistry;
};

class ComponentMask : public std::bitset<ComponentType::LIMIT_ID> {
public:
    bool is_match(const ComponentMask& filter) {
        return (*this & filter) == filter;
    }

    template<class... Ts>
    static ComponentMask pack() {
        ComponentMask result;
        (result.set(Ts::type_id()), ...);
        return result;
    }
};

template<class T>
class Component {
public:
    static void                 set_type_id(ComponentType::Id type_id);

    static ComponentType::Id    type_id();
    static const ComponentType& type();
};

class ComponentRegistry {
public:
    template<class T>
    ComponentType::Id register_component(String name) {
        auto type_id = m_type_id_counter++;
        SL_VERIFY(type_id < ComponentType::LIMIT_ID);
        SL_VERIFY(type_id != ComponentType::INVALID_ID);

        auto& type = m_types[type_id];
        SL_VERIFY(type.m_id == ComponentType::INVALID_ID);

        type.m_id = type_id;
        type.m_name = std::move(name);
        type.m_size = sizeof(T);

        type.m_constructor = [](void* buffer) {
            new (buffer) T();
        };

        type.m_destructor = [](void* buffer) {
            T* component = static_cast<T*>(buffer);
            std::string tname = typeid(*component).name();

            component->~T();
        };

        Component<T>::set_type_id(type_id);

        return type_id;
    }

    ComponentType::Id       limit_id() { return m_type_id_counter; }

    const ComponentType&    get_type(ComponentType::Id type_id) const { return m_types[type_id];}

    static ComponentRegistry& instance();

private:
    Array<ComponentType, ComponentType::LIMIT_ID>   m_types;
    ComponentType::Id                               m_type_id_counter = 0;
};

template<class T>
class ComponentRegistrator {
public:
    ComponentRegistrator(String name) {
        ComponentRegistry::instance().register_component<T>(std::move(name));
    }
};

#define REGISTER_COMPONENT(COMPONENT) \
static ComponentType::Id  __type_id_##COMPONENT = ComponentType::INVALID_ID; \
template<> ComponentType::Id Component<COMPONENT>::type_id()  { return __type_id_##COMPONENT; } \
template<> void Component<COMPONENT>::set_type_id(ComponentType::Id type_id) { \
    SL_VERIFY(type_id != ComponentType::INVALID_ID); \
    SL_VERIFY(__type_id_##COMPONENT == ComponentType::INVALID_ID); \
    __type_id_##COMPONENT = type_id; \
} \
static ComponentRegistrator<COMPONENT> __registrator_##COMPONENT(#COMPONENT)

class ComponentPool {
public:
    void*       acquire(size_t offset, size_t block_size);
    void*       get(size_t offset) { return m_pool.data() + offset; }
    const void* get(size_t offset) const { return m_pool.data() + offset; }
    size_t      size() const { return m_pool.size(); }

private:
    Vector<uint8_t> m_pool;
};

class ComponentManager : public ComponentPool {
public:
    explicit ComponentManager(const ComponentType* type)
        : m_comp_size(type->size())
        , m_comp_type(type) {}

    void* create_component(Entity e) {
        void* buffer = acquire(offset(e), m_comp_size);
        m_comp_type->create_component(buffer);
        return buffer;
    }

    void destroy_component(Entity e) {
        void* buffer = get(offset(e));
        m_comp_type->destroy_component(buffer);
    }

    void* get_component(Entity e) {
        return get(offset(e));
    }

    const void* get_component(Entity e) const {
        return get(offset(e));
    }

private:
    size_t offset(Entity e) const { return m_comp_size * e.id(); }

    const size_t            m_comp_size = 0;
    const ComponentType*    m_comp_type = nullptr;
};

template <class T>
inline const ComponentType& Component<T>::type() { return ComponentRegistry::instance().get_type(type_id()); }

} // slope::app

namespace std
{
template<> struct hash<slope::app::ComponentMask>
{
    std::size_t operator()(slope::app::ComponentMask const& mask) const noexcept
    {
        return std::hash<std::bitset<slope::app::ComponentType::LIMIT_ID>>()(mask);
    }
};
}