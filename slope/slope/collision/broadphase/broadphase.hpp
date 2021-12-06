#pragma once
#include "slope/collision/aabb.hpp"
#include "slope/containers/vector.hpp"

namespace slope {

template <class T>
class Broadphase {
public:
    using ProxyId = uint32_t;

    ProxyId add_proxy(T* data)
    {
        ProxyId new_id;
        if (!m_free_ids.empty()) {
            new_id = m_free_ids.back();
            m_free_ids.pop_back();

        } else {
            new_id = (ProxyId)m_proxies.size();
            m_proxies.emplace_back();
        }

        auto& proxy = m_proxies[new_id];
        proxy.data = data;
        proxy.active = true;
        return new_id;
    }

    void remove_proxy(ProxyId proxy_id)
    {
        m_proxies[proxy_id].active = false;
        m_free_ids.push_back(proxy_id);
    }

    void update_proxy(ProxyId proxy_id, const AABB& aabb)
    {
        m_proxies[proxy_id].aabb = aabb;
    }

    template<class Callback>
    void find_overlapping_pairs(const Callback& callback)
    {
        for (auto it1 = m_proxies.begin(); it1 != m_proxies.end(); it1++) {
            auto& p1 = *it1;
            if (!p1.active)
                continue;

            for (auto it2 = it1 + 1; it2 != m_proxies.end(); it2++) {
                auto& p2 = *it2;
                if (!p2.active)
                    continue;

                if (p1.aabb.intersects(p2.aabb))
                    callback(p1.data, p2.data);
            }
        }
    }

private:
    struct Proxy {
        T* data = nullptr;
        AABB aabb;
        bool active = false;
    };

    Vector<Proxy> m_proxies;
    Vector<ProxyId> m_free_ids;
};

} // slope
