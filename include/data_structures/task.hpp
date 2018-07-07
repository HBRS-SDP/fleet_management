#ifndef TASK_HPP
#define TASK_HPP

#include <string>
#include <vector>
#include <map>
#include <json/json.h>

namespace ccu
{
    struct Waypoint
    {
        std::string semantic_id;
        std::string area_id;
        int floor_number;
        int x;
        int y;

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
    };

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
    };

    struct Action
    {
        std::string id;
        std::vector<Area> areas;
        std::vector<Waypoint> waypoints;
        std::string execution_status; // pending, in progress, etc.
        float eta;

        Json::Value toJson() const
        {
            Json::Value action_json;
            action_json["id"] = id;
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

        static Action fromJson(std::string json_string)
        {
            Action action;
            return action;
        }
    };

    struct TaskRequest
    {
        Waypoint pickup_pose;
        Waypoint delivery_pose;
        float start_time;
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

    struct Task
    {
        int id;
        std::map<std::string, std::vector<Action>> robot_actions; //TODO: consider a different data structure for the list of actions
                                                                  //so that it's easier to expand it if necessary
        std::vector<std::string> team_robot_ids;
        float start_time;

        Json::Value toJson() const
        {
            Json::Value task_json;
            task_json["id"] = id;
            task_json["start_time"] = start_time;

            Json::Value &team_robot_id_list = task_json["team_robot_ids"];
            for (std::string robot_id : team_robot_ids)
            {
                team_robot_id_list.append(robot_id);
            }

            Json::Value &robot_action_list = task_json["robot_actions"];
            for (auto actions_per_robot : robot_actions)
            {
                std::string robot_id = actions_per_robot.first;
                std::vector<Action> actions = actions_per_robot.second;

                Json::Value action_list;
                for (Action action : actions)
                {
                    action_list.append(action.toJson());
                }

                Json::Value robot_action_list_json;
                robot_action_list_json[robot_id] = action_list;
                robot_action_list.append(robot_action_list_json);
            }

            return task_json;
        }

        static Task fromJson(std::string json_string)
        {
            Json::Value json_task;

            Json::CharReaderBuilder json_builder;
            Json::CharReader* json_reader = json_builder.newCharReader();
            std::string errors;
            bool parsingSuccessful = json_reader->parse(json_string.c_str(), json_string.c_str() + json_string.size(), &json_task, &errors);
            delete json_reader;

            Task task;
            task.id = json_task["id"].asInt();
            task.start_time = json_task["start_time"].asDouble();
            for (auto robot_id : json_task["team_robot_ids"])
            {
                std::string robot_id_str = robot_id.asString();
                task.team_robot_ids.push_back(robot_id_str);
            }

            Json::Value robot_action_list = json_task["robot_actions"];
            for (Json::Value::iterator robot_actions=robot_action_list.begin(); robot_actions!=robot_action_list.end(); ++robot_actions)
            {
                std::string robot_id = robot_actions.key().asString();
                task.robot_actions[robot_id] = std::vector<Action>();
                for (auto action_json : (*robot_actions))
                {
                    std::string action_str = action_json.asString();
                    Action action = Action::fromJson(action_str);
                    task.robot_actions[robot_id].push_back(action);
                }
            }

            return task;
        }
    };

    struct MissionStatus
    {
        int mission_id;
        std::map<int, Waypoint> robot_waypoints;
        std::map<int, Action> current_robot_actions;
        std::map<int, std::vector<Action>> completed_robot_actions;
        std::map<int, int> robot_floor;
        std::map<int, float> task_duration;
        std::map<int, float> mission_duration;
    };

    struct RobotTask
    {
        float start_time;
        float estimated_end_time;
    };

    struct RobotStatus
    {
        bool operational;
    };

    struct ElevatorRequests
    {
        int current_floor;
        int number_of_active_requests;
    };
}

#endif
