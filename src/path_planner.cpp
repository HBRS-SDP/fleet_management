#include "path_planner.hpp"

namespace ccu
{
    std::vector<Area> PathPlanner::getPathPlan(const Area& start_location, const Area& destination)
    {
        //TODO: find a path plan using OSM; add any necessary elevator calls to the list of plan actions
        Json::Value json_plan = this->generateTestOsmPlan(start_location, destination);
        std::vector<Area> path_plan = this->parsePlan(json_plan);
        path_plan.push_back(destination);
        return path_plan;
    }

    std::vector<Area> PathPlanner::parsePlan(Json::Value json_plan)
    {
        std::vector<Area> plan_areas;
        Json::Value &json_areas = json_plan["elements"];
        for (Json::Value json_area : json_areas)
        {
            Area area;
            area.id = json_area["tags"]["id"].asString();
            area.name = json_area["tags"]["name"].asString();
            area.floor_number = json_area["tags"]["floor_number"].asInt();
            plan_areas.push_back(area);
        }
        return plan_areas;
    }

    Json::Value PathPlanner::generateTestOsmPlan(const Area& start_location, const Area& destination)
    {
        Json::Value plan;
        return plan;
    }
}
