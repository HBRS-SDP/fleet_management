#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include <vector>
#include "data_structures/task.hpp"

namespace ccu
{
    class PathPlanner
    {
    public:
        static std::vector<Area> getPathPlan(const Area& start_location, const Area& destination);
    private:
        static std::vector<Area> parsePlan(Json::Value json_plan);
        static Json::Value generateTestOsmPlan(const Area& start_location, const Area& destination);
    };
}

#endif
