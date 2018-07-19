#ifndef TASK_REQUEST_HPP
#define TASK_REQUEST_HPP

#include <string>
#include <json/json.h>

#include "data_structures/waypoint.hpp"

namespace ccu
{
    struct TaskRequest
    {
        Area pickup_pose;
        Area delivery_pose;
        double start_time;
        std::string user_id;
        std::string cart_type;
        std::string cart_id;

        Json::Value toJson() const
        {
            Json::Value task_request_json;
            task_request_json["pickup_pose"] = pickup_pose.toJson();
            task_request_json["delivery_pose"] = delivery_pose.toJson();
            task_request_json["start_time"] = start_time;
            task_request_json["user_id"] = user_id;
            task_request_json["cart_type"] = cart_type;
            task_request_json["cart_id"] = cart_id;
            return task_request_json;
        }
    };
};

#endif