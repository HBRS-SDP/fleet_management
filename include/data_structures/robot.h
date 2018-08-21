#ifndef ROBOT_H
#define ROBOT_H

#include <string>

#include "data_structures/robot_status.hpp"


namespace ccu
{
    struct Robot
    {
        std::string robot_id;
        std::string schedule;
        RobotStatus status;

        Json::Value toJson() const
        {
            Json::Value robot_json;
            robot_json["robot_id"] = robot_id;
            //TODO: Add schedule and robot status to resources
            return robot_json;
        }

    };
}

#endif //ROBOT_H
