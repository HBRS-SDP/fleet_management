#ifndef TASK_EXECUTOR_HPP
#define TASK_EXECUTOR_HPP

#include "data_structures/task.hpp"

namespace task
{
    class TaskExecutor
    {
    public:
        TaskExecutor() { }
        ~TaskExecutor() { }

        bool executeTask(const Task& task);
    };
}

#endif
