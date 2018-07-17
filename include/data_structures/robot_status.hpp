#ifndef ROBOT_STATUS_HPP
#define ROBOT_STATUS_HPP

#include <json/json.h>

#include "data_structures/area.hpp"

namespace ccu
{
    struct RobotStatus
    {
        ccu::Area current_location;
        bool operational;

        Json::Value toJson() const
        {
            Json::Value status_json;
            status_json["current_location"] = current_location.toJson();
            status_json["operational"] = operational;
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
            status.current_location = Area::fromJson(status_json["current_location"]);
            status.operational = status_json["operational"].asBool();
            return status;
        }
    };
}

#endif