#include "slope/flow/task_executor.hpp"
#include "slope/core/config.hpp"
#include "slope/core/string.hpp"
#include "slope/core/vector.hpp"
#include "taskflow/taskflow.hpp"
#include <memory>


namespace slope {

class ParallelExecutor : public TaskExecutor {
public:
    explicit ParallelExecutor(int concurrency = 4)
        : m_concurrency(concurrency)
        , m_tf_executor(std::make_unique<tf::Executor>(concurrency)) {}

    TaskId emplace(Callback&& task, std::string_view name) final
    {
#ifdef SL_TRACY_ENABLE
        auto tf_task = m_taskflow.emplace([task = std::move(task), name = String(name)]() {
            SL_ZONE_SCOPED_DYNAMIC(name.c_str(), name.size())
            task();
        });
#else
        auto tf_task = m_taskflow.emplace(std::move(task));
#endif
        m_tasks.push_back(tf_task);
        m_tasks.back().name(std::string{name});
        return static_cast<TaskId>(m_tasks.size() - 1);
    }

    void set_order(TaskId pre, TaskId post) final
    {
        m_tasks[pre].precede(m_tasks[post]);
    }

    int concurrency() const final { return m_concurrency; }

    tf::Future<void> run()
    {
        return m_tf_executor->run(m_taskflow);
    }

private:
    std::unique_ptr<tf::Executor> m_tf_executor;
    tf::Taskflow m_taskflow;

    Vector<tf::Task> m_tasks;

    int m_concurrency = 0;
};

} // namespace slope
