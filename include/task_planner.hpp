#ifndef TASK_PLANNER_HPP
#define TASK_PLANNER_HPP

#include <vector>
#include "data_structures/task.hpp"

namespace task
{
    class TaskPlanner
    {
    public:
        TaskPlanner() { }
        ~TaskPlanner() { }

        std::vector<Action> getTaskPlan(const TaskRequest& task_request) const;
    };
}

#endif
