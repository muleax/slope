#pragma once
#include "slope/thread/task_executor.hpp"
#include "slope/containers/vector.hpp"
#include <cstdint>

namespace slope {

class AsyncExecutor : public TaskExecutor {
public:
    TaskId emplace(Callback&& task) final
    {
        m_tasks.push_back({std::move(task)});
        return static_cast<TaskId>(m_tasks.size() - 1);
    }

    void set_order(TaskId pre, TaskId post) final
    {
        m_edges.push_back({pre, post});
    }

    int concurrency() const final { return 1; }

    void run()
    {
        m_tick++;

        size_t executed_cnt = 0;
        while (executed_cnt < m_tasks.size()) {
            for (TaskId task = 0; task < m_tasks.size(); task++) {
                auto& data = m_tasks[task];
                if (data.tick != m_tick && check_dependencies(task)) {
                    data.task();
                    data.tick = m_tick;
                    executed_cnt++;
                }
            }
        }
    }

private:
    struct TaskData {
        Callback task;
        uint64_t tick = 0;
    };

    struct Edge {
        TaskId pre = 0;
        TaskId post = 0;
    };

    bool check_dependencies(TaskId task)
    {
        return std::all_of(m_edges.begin(), m_edges.end(),
                    [task, this] (auto& edge) { return edge.post != task || m_tasks[edge.pre].tick == m_tick; });
    }

    Vector<TaskData> m_tasks;
    Vector<Edge> m_edges;

    uint64_t m_tick = 0;
};

} // slope
