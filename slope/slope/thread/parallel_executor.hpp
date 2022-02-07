#include "task_executor.hpp"
#include "slope/thread/task_executor.hpp"
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
        m_tasks.push_back(m_taskflow.emplace(std::move(task)));
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
