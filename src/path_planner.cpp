#include "path_planner.hpp"

namespace ccu
{
    std::vector<Area> PathPlanner::getPathPlan(const Area& start_location, const Area& destination)
    {
        //TODO: find a path plan using OSM; add any necessary elevator calls to the list of plan actions
        Json::Value json_plan = PathPlanner::generateTestOsmPlan(start_location, destination);
        std::vector<Area> path_plan = PathPlanner::parsePlan(json_plan);
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
        Json::Value &elements = plan["elements"];

        if (start_location.name == "pickup_location" && destination.name == "delivery_location")
        {
            Json::Value wp1;
            wp1["tags"]["id"] = "1";
            wp1["tags"]["name"] = "hallway1";
            wp1["tags"]["floor_number"] = 0;
            elements.append(wp1);

            Json::Value wp2;
            wp2["tags"]["id"] = "2";
            wp2["tags"]["name"] = "hallway2";
            wp2["tags"]["floor_number"] = 0;
            elements.append(wp2);

            Json::Value wp3;
            wp3["tags"]["id"] = "3";
            wp3["tags"]["name"] = "hallway3";
            wp3["tags"]["floor_number"] = 0;
            elements.append(wp3);
        }
        else if (start_location.name == "delivery_location" && destination.name == "charging_station")
        {
            Json::Value wp1;
            wp1["tags"]["id"] = "1";
            wp1["tags"]["name"] = "hallway3";
            wp1["tags"]["floor_number"] = 0;
            elements.append(wp1);

            Json::Value wp2;
            wp2["tags"]["id"] = "2";
            wp2["tags"]["name"] = "hallway2";
            wp2["tags"]["floor_number"] = 0;
            elements.append(wp2);

            Json::Value wp3;
            wp3["tags"]["id"] = "3";
            wp3["tags"]["name"] = "hallway1";
            wp3["tags"]["floor_number"] = 0;
            elements.append(wp3);
        }
        return plan;
    }
}
