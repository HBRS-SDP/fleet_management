#ifndef ROBOT_STATUS_HPP
#define ROBOT_STATUS_HPP

#include <string>
#include <json/json.h>

#include "data_structures/area.hpp"

namespace ccu
{
    struct RobotStatus
    {
        std::string robot_id;
        ccu::Area current_location;
        std::string current_operation;
        std::string status;
        bool available;
        double battery_status;

        Json::Value toJson() const
        {
            Json::Value status_json;
            status_json["robot_id"] = robot_id;
            status_json["current_location"] = current_location.toJson();
            status_json["current_operation"] = current_operation;
            status_json["status"] = status;
            status_json["available"] = available;
            status_json["battery_status"] = battery_status;
            return status_json;
        }

        static RobotStatus fromJson(const std::string &json_string)
        {
            Json::Value status_json;
            Json::CharReaderBuilder json_builder;
            Json::CharReader* json_reader = json_builder.newCharReader();
            std::string errors;
            bool parsingSuccessful = json_reader->parse(json_string.c_str(), json_string.c_str() + json_string.size(), &status_json, &errors);
            delete json_reader;

            RobotStatus status;
            status.robot_id = status_json["robot_id"].asString();
            status.current_location = Area::fromJson(status_json["current_location"]);
            status.current_operation = status_json["current_operation"].asString();
            status.status = status_json["status"].asString();
            status.available = status_json["available"].asBool();
            status.battery_status = status_json["battery_status"].asDouble();
            return status;
        }
    };
}

#endif