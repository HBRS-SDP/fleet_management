#ifndef TASK_PLANNER_HPP
#define TASK_PLANNER_HPP

#include <vector>

#include "data_structures/action.hpp"
#include "data_structures/task_request.hpp"
#include "path_planner.hpp"

namespace ccu
{
    class TaskPlanner
    {
    public:
        static std::vector<Action> getTaskPlan(const TaskRequest& task_request);
    private:
        static std::vector<Action> expandTaskPlan(const std::vector<Action>& task_plan);
    };
}

#endif
