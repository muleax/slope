#pragma once
#include "slope/collision/aabb.hpp"
#include "slope/containers/vector.hpp"
#include <algorithm>

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

    template<class Callback>
    void find_overlapping_pairs_SP(const Callback& callback)
    {
        float mean0 = 0.f;
        float mean1 = 0.f;
        float mean2 = 0.f;

        m_order.clear();
        for (ProxyId i = 0; i < m_proxies.size(); i++) {
            auto& p = m_proxies[i];
            if (p.active) {
                m_order.push_back({p.data, p.aabb});
                mean0 += p.aabb.min.x;
                mean1 += p.aabb.min.y;
                mean2 += p.aabb.min.z;
            }
        }

        mean0 /= m_order.size();
        mean1 /= m_order.size();
        mean2 /= m_order.size();

        float var0 = 0.f;
        float var1 = 0.f;
        float var2 = 0.f;

        for (auto& data : m_order) {
            auto& v = data.aabb.min;
            var0 += sqr(v.x - mean0);
            var1 += sqr(v.y - mean1);
            var2 += sqr(v.z - mean2);
        }

        int axis = 0;
        float max_var = var0;

        if (var1 > max_var) {
            max_var = var1;
            axis = 1;
        }

        if (var2 > max_var) {
            max_var = var2;
            axis = 2;
        }

        std::sort(m_order.begin(), m_order.end(), [this, axis](auto& a, auto& b) {
            return a.aabb.min[axis] < b.aabb.min[axis];
        });

        for (int i = 0; i < m_order.size(); i++) {
            auto& p1 = m_order[i];
            for (int j = i + 1; j < m_order.size(); j++) {
                auto& p2 = m_order[j];
                if (p1.aabb.max[axis] >= p2.aabb.min[axis]) {
                    if (p1.aabb.intersects(p2.aabb))
                        callback(p1.data, p2.data);
                } else {
                    break;
                }
            }
        }
    }

private:
    struct Proxy {
        T* data = nullptr;
        AABB aabb;
        bool active = false;
    };

    struct Data {
        T* data = nullptr;
        AABB aabb;
    };

    Vector<Proxy> m_proxies;
    Vector<ProxyId> m_free_ids;

    Vector<Data> m_order;
};

} // slope
