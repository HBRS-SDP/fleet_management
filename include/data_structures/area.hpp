#ifndef AREA_HPP
#define AREA_HPP

#include <string>
#include <vector>
#include <json/json.h>

#include "data_structures/waypoint.hpp"

namespace ccu
{
    struct Area
    {
        std::string id;
        std::vector<Waypoint> waypoints;

        Json::Value toJson() const
        {
            Json::Value action_json;
            action_json["id"] = id;
            Json::Value &waypoint_list = action_json["waypoints"];
            for (Waypoint waypoint : waypoints)
            {
                Json::Value waypoint_json = waypoint.toJson();
                waypoint_list.append(waypoint_json);
            }

            return action_json;
        }

        static Area fromJson(const Json::Value &area_json)
        {
            Area area;
            area.id = area_json["id"].asString();
            const Json::Value &wp_list = area_json["waypoints"];
            for (int i = 0; i < wp_list.size(); i++)
            {
                Waypoint wp = Waypoint::fromJson(wp_list[i]);
                area.waypoints.push_back(wp);
            }
            return area;
        }
    };
};

#endif