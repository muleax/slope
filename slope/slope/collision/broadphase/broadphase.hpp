#pragma once
#include "slope/collision/aabb.hpp"
#include "slope/core/vector.hpp"
#include "slope/thread/utils.hpp"
#include "slope/debug/log.hpp"
#include <algorithm>

namespace slope {

template <class T>
class Broadphase {
public:
    using ProxyId = uint32_t;

    struct Overlap {
        T data1;
        T data2;
    };

    ProxyId add_proxy(T data);
    void    remove_proxy(ProxyId proxy_id);
    void    update_proxy(ProxyId proxy_id, const AABB& aabb);

    void    clear();

    void    set_concurrency(int concurrency) { m_concurrency = concurrency; }
    int     concurrency() const { return m_concurrency; }

    void    find_overlaps_pass0();
    void    find_overlaps_pass1(int worker_id, Vector<Overlap>& out_overlaps);

private:
    struct Proxy {
        bool    active = false;
        T       data;
        AABB    aabb;
    };

    struct TraverseData {
        T       data;
        AABB    aabb;
    };

    Vector<Proxy>           m_proxies;
    Vector<ProxyId>         m_free_ids;
    int                     m_concurrency = 1;

    Vector<TraverseData>    m_traverse_order;
    int                     m_sort_axis = 0;
};

template <class T>
typename Broadphase<T>::ProxyId Broadphase<T>::add_proxy(T data)
{
    ProxyId new_id;
    if (!m_free_ids.empty()) {
        new_id = m_free_ids.back();
        m_free_ids.pop_back();

        auto& proxy = m_proxies[new_id];
        proxy.active = true;
        proxy.data = data;

    } else {
        new_id = static_cast<ProxyId>(m_proxies.size());
        m_proxies.push_back({true, data});
    }

    return new_id;
}

template <class T>
void Broadphase<T>::remove_proxy(typename Broadphase<T>::ProxyId proxy_id)
{
    m_proxies[proxy_id].active = false;
    m_free_ids.push_back(proxy_id);
}

template <class T>
void Broadphase<T>::clear()
{
    m_proxies.clear();
    m_free_ids.clear();
}

template <class T>
void Broadphase<T>::update_proxy(typename Broadphase<T>::ProxyId proxy_id, const AABB& aabb)
{
    m_proxies[proxy_id].aabb = aabb;
}

template <class T>
void Broadphase<T>::find_overlaps_pass0()
{
    float mean0 = 0.f;
    float mean1 = 0.f;
    float mean2 = 0.f;

    m_traverse_order.clear();
    for (ProxyId i = 0; i < m_proxies.size(); i++) {
        auto& p = m_proxies[i];
        if (p.active) {
            m_traverse_order.push_back({p.data, p.aabb});
            mean0 += p.aabb.min.x;
            mean1 += p.aabb.min.y;
            mean2 += p.aabb.min.z;
        }
    }

    mean0 /= m_traverse_order.size();
    mean1 /= m_traverse_order.size();
    mean2 /= m_traverse_order.size();

    float var0 = 0.f;
    float var1 = 0.f;
    float var2 = 0.f;

    for (auto& data : m_traverse_order) {
        auto& v = data.aabb.min;
        var0 += sqr(v.x - mean0);
        var1 += sqr(v.y - mean1);
        var2 += sqr(v.z - mean2);
    }

    // find max variance axis
    m_sort_axis = 0;
    float max_var = var0;

    if (var1 > max_var) {
        max_var = var1;
        m_sort_axis = 1;
    }

    if (var2 > max_var) {
        max_var = var2;
        m_sort_axis = 2;
    }

    std::sort(m_traverse_order.begin(), m_traverse_order.end(), [this](auto& a, auto& b) {
        return a.aabb.min[m_sort_axis] < b.aabb.min[m_sort_axis];
    });
}

template <class T>
void Broadphase<T>::find_overlaps_pass1(int worker_id, Vector<Overlap>& out_overlaps)
{
    auto [chunk_beg, chunk_end] = select_sequence_chunk(worker_id, m_concurrency, m_traverse_order);

    for (auto proxy1 = chunk_beg; proxy1 != chunk_end; ++proxy1) {
        for (auto proxy2 = proxy1 + 1; proxy2 != m_traverse_order.end(); ++proxy2) {
            if (proxy1->aabb.max[m_sort_axis] >= proxy2->aabb.min[m_sort_axis]) {
                if (proxy1->aabb.intersects(proxy2->aabb)) {
                    out_overlaps.push_back({proxy1->data, proxy2->data});
                }
            } else {
                break;
            }
        }
    }
}

} // slope
