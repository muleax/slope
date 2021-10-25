#pragma once
#include "slope/containers/unordered_map.hpp"
#include "slope/containers/string.hpp"
#include <memory>
#include <functional>

namespace slope::app {

template <class T>
class ResourceCache {
public:
    using ResourcePtr = std::shared_ptr<T>;

    template <class... Keys>
    ResourcePtr get(const Keys&... keys) {
        auto hash = make_hash(keys...);
        auto it = m_resources.find(hash);
        if (it != m_resources.end()) {
            return it->second;
        }

        return nullptr;
    }

    template <class... Keys>
    void put(ResourcePtr resource, const Keys&... keys) {
        m_resources[make_hash(keys...)] = std::move(resource);
    }

protected:
    using Hash = size_t;

    template <class... Keys>
    Hash make_hash(const Keys&... keys) {
        Hash hash = 0;
        ((hash = hash ^ (std::hash<Keys>()(keys) + 0x9e3779b9 + (hash << 6) + (hash >> 2))), ...);
        return hash;
    }

    template <class... Keys>
    bool save_impl(const std::function<bool()>& saver, ResourcePtr resource, const Keys&... keys) {
        bool success = saver();
        if (success) {
            put(resource, keys...);
        }

        return success;
    }

    template <class... Keys>
    ResourcePtr load_impl(const std::function<ResourcePtr()>& loader, const Keys&... keys) {
        ResourcePtr cached = get(keys...);
        if (cached) {
            return cached;
        }

        ResourcePtr loaded = loader();
        if (loaded) {
            put(loaded, keys...);
        }

        return loaded;
    }

    UnorderedMap<Hash, ResourcePtr> m_resources;
};

} // slope::app