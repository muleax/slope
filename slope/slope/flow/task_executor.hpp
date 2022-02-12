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

    template<class... Ts>
    void            set_order(TaskId a, TaskId b, Ts... tail)
    {
        set_order(a, b);
        set_order(b, tail...);
    }
};

} // slope
