#pragma once
#include "slope/collision/aabb.hpp"
#include "slope/containers/vector.hpp"
#include "slope/thread/task_executor.hpp"
#include "slope/debug/log.hpp"
#include <algorithm>
#include <optional>

namespace slope {

template <class T>
class Broadphase {
public:
    using ProxyId = uint32_t;

    ProxyId add_proxy(T* data);
    void    remove_proxy(ProxyId proxy_id);
    void    update_proxy(ProxyId proxy_id, const AABB& aabb);

    // Sweep and prune
    template <class Callback>
    void    setup_executor(
        const Callback& callback, TaskExecutor& executor, int concurrency,
        std::optional<TaskId> pre_fence, std::optional<TaskId> post_fence);

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

    struct TaskContext {
        int chunk_beg = 0;
        int chunk_end = 0;
    };

    void update_traverse_order();
    void update_execution_context();

    Vector<Proxy>           m_proxies;
    Vector<ProxyId>         m_free_ids;
    Vector<Data>            m_traverse_order;
    Vector<TaskContext>     m_task_ctx;
    int                     m_sort_axis = 0;
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
void Broadphase<T>::update_traverse_order()
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
void Broadphase<T>::update_execution_context()
{
    update_traverse_order();

    int concurrency = static_cast<int>(m_task_ctx.size());
    size_t chunk_beg = 0;

    for (int task_idx = 0; task_idx < concurrency; task_idx++) {
        auto& ctx = m_task_ctx[task_idx];
        ctx.chunk_beg = chunk_beg;
        ctx.chunk_end = (task_idx == concurrency - 1)
                        ? m_traverse_order.size()
                        : ((task_idx + 1) * m_traverse_order.size()) / concurrency;

        chunk_beg = ctx.chunk_end;
    }
}

template <class T>
template <class Callback>
void Broadphase<T>::setup_executor(
    const Callback& callback, TaskExecutor& executor, int concurrency,
    std::optional<TaskId> pre_fence, std::optional<TaskId> post_fence)
{
    m_task_ctx.resize(concurrency);

    auto prepare_task = executor.emplace([this]() {
        update_execution_context();
    });

    if (pre_fence.has_value())
        executor.set_order(*pre_fence, prepare_task);

    for (int task_idx = 0; task_idx < concurrency; task_idx++) {

        auto chunk_task = executor.emplace([task_idx, callback, this]() {
            auto& ctx = m_task_ctx[task_idx];
            for (size_t i = ctx.chunk_beg; i < ctx.chunk_end; i++) {
                auto& proxy1 = m_traverse_order[i];
                for (size_t j = i + 1; j < m_traverse_order.size(); j++) {
                    auto& proxy2 = m_traverse_order[j];
                    if (proxy1.aabb.max[m_sort_axis] >= proxy2.aabb.min[m_sort_axis]) {
                        if (proxy1.aabb.intersects(proxy2.aabb)) {
                            callback(proxy1.data, proxy2.data, task_idx);
                        }
                    } else {
                        break;
                    }
                }
            }
        });

        executor.set_order(prepare_task, chunk_task);
        if (post_fence.has_value())
            executor.set_order(chunk_task, *post_fence);
    }
}

} // slope
