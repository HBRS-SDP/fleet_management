#include "path_planner.hpp"

namespace ccu
{
    std::vector<Action> PathPlanner::expandTaskPlan(const std::vector<Action>& task_plan)
    {
        std::vector<Action> expanded_task_plan;
        for (Action action : task_plan)
        {
            if (action.id == "go_to")
            {
                std::vector<Action> path_plan = getPathPlan(action);
                expanded_task_plan.reserve(expanded_task_plan.size() + path_plan.size());
                expanded_task_plan.insert(expanded_task_plan.end(), path_plan.begin(), path_plan.end());
            }
            else
            {
                expanded_task_plan.push_back(action);
            }
        }
        return expanded_task_plan;
    }

    std::vector<Action> PathPlanner::getPathPlan(const Action& go_to_action)
    {
        std::vector<Action> path_plan;
        Area destination = go_to_action.areas[0];
        //TODO: find a path plan using OSM; add any necessary elevator calls to the list of plan actions
        path_plan.push_back(go_to_action);
        return path_plan;
    }
}
