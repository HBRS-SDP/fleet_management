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

        std::vector<Area> getPathPlan(const Area& start_location, const Area& destination);
    private:
        std::vector<Area> parsePlan(Json::Value json_plan);
        Json::Value generateTestOsmPlan(const Area& start_location, const Area& destination);
    };
}

#endif
