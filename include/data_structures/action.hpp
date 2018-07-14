#ifndef ACTION_HPP
#define ACTION_HPP

#include <string>
#include <vector>
#include <json/json.h>

#include "data_structures/waypoint.hpp"
#include "data_structures/area.hpp"

namespace ccu
{
    struct Action
    {
        std::string id;
        std::string type;
        std::vector<Area> areas;
        std::vector<Waypoint> waypoints;
        std::string execution_status; // pending, in progress, etc.
        float eta;

        Json::Value toJson() const
        {
            Json::Value action_json;
            action_json["id"] = id;
            action_json["type"] = type;
            action_json["execution_status"] = execution_status;
            action_json["eta"] = eta;

            Json::Value &area_list = action_json["areas"];
            for (Area area : areas)
            {
                Json::Value area_json = area.toJson();
                area_list.append(area_json);
            }

            Json::Value &waypoint_list = action_json["waypoints"];
            for (Waypoint waypoint : waypoints)
            {
                Json::Value waypoint_json = waypoint.toJson();
                waypoint_list.append(waypoint_json);
            }

            return action_json;
        }

        static Action fromJson(const Json::Value &action_json)
        {
            Action action;
            action.id = action_json["id"].asString();
            action.type = action_json["type"].asString();
            action.execution_status = action_json["execution_status"].asString();
            action.eta = action_json["eta"].asFloat();
            const Json::Value &area_list = action_json["areas"];
            for (int i = 0; i < area_list.size(); i++)
            {
                Area area = Area::fromJson(area_list[i]);
                action.areas.push_back(area);
            }
            const Json::Value &wp_list = action_json["waypoints"];
            for (int i = 0; i < wp_list.size(); i++)
            {
                Waypoint wp = Waypoint::fromJson(wp_list[i]);
                action.waypoints.push_back(wp);
            }
            return action;
        }
    };
};

#endif