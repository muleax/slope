#pragma once
#include "slope/core/vector.hpp"
#include <cstdint>

namespace slope {

class DisjointSet {
public:
    explicit DisjointSet(uint32_t size = 0)
    {
        reset(size);
    }

    void        reset(uint32_t size)
    {
        m_nodes.resize(size);
        uint32_t node = 0;
        for (auto& data : m_nodes) {
            data.parent = node++;
            data.size = 1;
        }
    }

    uint32_t    find_root(uint32_t node)
    {
        auto& data = m_nodes[node];
        if (data.parent != node)
            data.parent = find_root(data.parent);

        return data.parent;
    }

    void        merge(uint32_t node1, uint32_t node2)
    {
        uint32_t root1 = find_root(node1);
        uint32_t root2 = find_root(node2);

        if (root1 == root2)
            return;

        auto* data1 = &m_nodes[root1];
        auto* data2 = &m_nodes[root2];

        if (data1->size < data2->size) {
            std::swap(data1, data2);
            std::swap(root1, root2);
        }

        data2->parent = root1;
        data1->size += data2->size;
    }

    uint32_t    get_size(uint32_t root) const
    {
        return m_nodes[root].size;
    }

private:
    struct Node {
        uint32_t parent = 0;
        uint32_t size = 0;
    };

    Vector<Node> m_nodes;
};

} // slope
