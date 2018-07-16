#ifndef TASK_PLANNER_HPP
#define TASK_PLANNER_HPP

#include <vector>

#include "data_structures/action.hpp"
#include "data_structures/task_request.hpp"

namespace ccu
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
