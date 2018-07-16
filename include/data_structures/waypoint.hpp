#ifndef WAYPOINT_HPP
#define WAYPOINT_HPP

#include <string>
#include <json/json.h>

namespace ccu
{
    struct Waypoint
    {
        std::string semantic_id;
        std::string area_id;
        int floor_number;
        double x;
        double y;

        Json::Value toJson() const
        {
            Json::Value waypoint_json;
            waypoint_json["semantic_id"] = semantic_id;
            waypoint_json["area_id"] = area_id;
            waypoint_json["floor_number"] = floor_number;
            waypoint_json["x"] = x;
            waypoint_json["y"] = y;
            return waypoint_json;
        }

        static Waypoint fromJson(const Json::Value &wp_json)
        {
            Waypoint wp;
            wp.semantic_id = wp_json["semantic_id"].asString();
            wp.area_id = wp_json["area_id"].asString();
            wp.floor_number = wp_json["floor_number"].asInt();
            wp.x = wp_json["x"].asDouble();
            wp.y = wp_json["y"].asDouble();
            return wp;
        }
    };
};

#endif