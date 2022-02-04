#pragma once
#include <functional>

namespace slope {

using TaskId = int;

class TaskExecutor {
public:
    using Callback = std::function<void()>;

    virtual ~TaskExecutor() = default;

    virtual TaskId  emplace(Callback&& task) = 0;
    virtual void    set_order(TaskId pre, TaskId post) = 0;

    virtual int     concurrency() const = 0;
};

} // slope
