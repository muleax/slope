#pragma once
#include <functional>
#include <string>

namespace slope {

using TaskId = int;

class TaskExecutor {
public:
    using Callback = std::function<void()>;

    virtual ~TaskExecutor() = default;

    virtual TaskId  emplace(Callback&& task, std::string_view name) = 0;
    virtual void    set_order(TaskId pre, TaskId post) = 0;

    virtual int     concurrency() const = 0;

    void            set_order(TaskId a, TaskId b, TaskId c)
    {
        set_order(a, b);
        set_order(b, c);
    }

    void            set_order(TaskId a, TaskId b, TaskId c, TaskId d)
    {
        set_order(a, b);
        set_order(b, c);
        set_order(c, d);
    }
};

} // slope
