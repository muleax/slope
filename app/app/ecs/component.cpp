#include "app/ecs/component.hpp"
#include "slope/debug/assert.hpp"

namespace slope::app {

ComponentRegistry& ComponentRegistry::instance() {
    static ComponentRegistry s_instance;
    return s_instance;
}

void* ComponentPool::acquire(size_t offset, size_t block_size) {
    if (offset + block_size > m_pool.size()) {
        m_pool.resize((offset + block_size) * 2);
    }

    return get(offset);
}

} // slope::app