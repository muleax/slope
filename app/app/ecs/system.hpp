#pragma once
#include "app/ecs/world.hpp"
#include "slope/core/vector.hpp"
#include "slope/debug/assert.hpp"

namespace slope::app {

class System {
public:
    explicit System(World* world) : m_world(world) {
        SL_VERIFY(world != nullptr);
    }
    virtual ~System() = default;

    virtual void update(float dt) = 0;

    World& w() { return *m_world; }

    template <class... Ts>
    const EntityView& view() const { return m_world->view<Ts...>(); }

private:
    World* m_world = nullptr;
};

} // slope::app
