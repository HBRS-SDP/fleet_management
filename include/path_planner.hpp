#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include <vector>
#include "data_structures/task.hpp"

namespace ccu
{
    class PathPlanner
    {
    public:
        PathPlanner() { }
        ~PathPlanner() { }

        std::vector<Action> expandTaskPlan(const std::vector<Action>& task_plan);
    private:
        std::vector<Action> getPathPlan(const Action& go_to_action);
    };
}

#endif
