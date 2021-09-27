#pragma once
#include "slope/containers/unordered_map.hpp"
#include "slope/containers/string.hpp"
#include <istream>
#include <fstream>
#include <memory>

namespace slope {

class IResource
{
public:
    virtual ~IResource() = default;
    virtual bool load(const String& path) = 0;
};

template <class T>
class ResourcePtr {
public:
    ResourcePtr() : m_ptr(nullptr) {}
    ResourcePtr(T* ptr) : m_ptr(ptr) {}
    ResourcePtr(const ResourcePtr& other) : m_ptr(other.m_ptr) {}

    ResourcePtr& operator=(const ResourcePtr& other) { m_ptr = other.m_ptr; return *this; }

    T* operator->() { return m_ptr; }
    const T* operator->() const { return m_ptr; }

    T& operator*() { return *m_ptr; }
    const T& operator*() const { return *m_ptr; }

    bool operator==(const ResourcePtr& other) const { return m_ptr == other.m_ptr; }
    bool operator==(const T* ptr) const { return m_ptr == ptr; }

    operator bool() const { return m_ptr != nullptr; }

private:
    T* m_ptr = nullptr;
};

class ResourceManager {
public:
    ResourceManager();

    template<class TResource>
    ResourcePtr<TResource> acquire(const String& path) {
        static_assert(std::is_base_of_v<IResource, TResource>);

        auto it = m_resources.find(path);
        if (it != m_resources.end())
        {
            return static_cast<TResource*>(it->second.get());
        }

        auto resource = std::make_unique<TResource>();
        if (!load_impl(path, *resource))
        {
            return nullptr;
        }

        return static_cast<TResource*>((m_resources[path] = std::move(resource)).get());
    }

    static ResourceManager& instance() { return m_instance; }

private:
    bool load_impl(const String& path, IResource& resource);

    slope::UnorderedMap<slope::String, std::unique_ptr<IResource>> m_resources;

    static ResourceManager m_instance;
};

} // slope