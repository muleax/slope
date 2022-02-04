#pragma once
#include "slope/collision/aabb.hpp"
#include "slope/containers/vector.hpp"
#include "slope/debug/log.hpp"
#include <algorithm>

#include "taskflow/taskflow.hpp"

namespace slope {

template <class T>
class Broadphase {
public:
    using ProxyId = uint32_t;

    ProxyId add_proxy(T* data);
    void    remove_proxy(ProxyId proxy_id);
    void    update_proxy(ProxyId proxy_id, const AABB& aabb);

    void    set_concurrency(int concurrency) { m_concurrency = concurrency; }
    int     concurrency() const { return m_concurrency; }

    // Sweep and prune
    template<class Callback>
    void    traverse_overlapping_pairs(Callback&& callback, tf::Taskflow& flow, const tf::Task& fence);

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
    int m_concurrency = 8;
};

template <class T>
typename Broadphase<T>::ProxyId Broadphase<T>::add_proxy(T* data)
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

template <class T>
void Broadphase<T>::remove_proxy(typename Broadphase<T>::ProxyId proxy_id)
{
    m_proxies[proxy_id].active = false;
    m_free_ids.push_back(proxy_id);
}

template <class T>
void Broadphase<T>::update_proxy(typename Broadphase<T>::ProxyId proxy_id, const AABB& aabb)
{
    m_proxies[proxy_id].aabb = aabb;
}

template <class T>
template <class Callback>
void Broadphase<T>::traverse_overlapping_pairs(Callback&& callback, tf::Taskflow& flow, const tf::Task& fence)
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

    // find max variance axis
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

    size_t chunk_beg = 0;

    for (int worker_id = 0; worker_id < m_concurrency; worker_id++) {
        size_t chunk_end = m_order.size();
        if (worker_id < m_concurrency - 1)
            chunk_end = ((worker_id + 1) * m_order.size()) / m_concurrency;

        flow.template emplace([chunk_beg, chunk_end, axis, worker_id, callback, this]() {
            for (size_t i = chunk_beg; i < chunk_end; i++) {
                auto& p1 = m_order[i];
                for (size_t j = i + 1; j < m_order.size(); j++) {
                    auto& p2 = m_order[j];
                    if (p1.aabb.max[axis] >= p2.aabb.min[axis]) {
                        if (p1.aabb.intersects(p2.aabb))
                            callback(p1.data, p2.data, worker_id);
                    } else {
                        break;
                    }
                }
            }
        }).precede(fence);

        chunk_beg = chunk_end;
    }
}

} // slope
