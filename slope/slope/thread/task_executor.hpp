#pragma once
#include <functional>
#include <optional>

namespace slope {

using TaskId = int;

struct Fence {
    TaskId pre = 0;
    TaskId post = 0;
};

class TaskExecutor {
public:
    using Callback = std::function<void()>;

    virtual ~TaskExecutor() = default;

    virtual TaskId  emplace(Callback&& task, const char* name = nullptr) = 0;
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
