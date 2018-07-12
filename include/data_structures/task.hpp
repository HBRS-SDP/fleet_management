#ifndef TASK_HPP
#define TASK_HPP

#include <string>
#include <vector>
#include <map>
#include <json/json.h>
#include <sstream>

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

    struct TaskRequest
    {
        Waypoint pickup_pose;
        Waypoint delivery_pose;
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

    struct Task
    {
        std::string id;
        std::map<std::string, std::vector<Action>> robot_actions; //TODO: consider a different data structure for the list of actions
                                                                  //so that it's easier to expand it if necessary
        std::vector<std::string> team_robot_ids;
        double start_time;

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

        static Task fromJson(const std::string &json_string)
        {
            Json::Value json_task;

            Json::CharReaderBuilder json_builder;
            Json::CharReader* json_reader = json_builder.newCharReader();
            std::string errors;
            bool parsingSuccessful = json_reader->parse(json_string.c_str(), json_string.c_str() + json_string.size(), &json_task, &errors);
            delete json_reader;

            Task task;
            task.id = json_task["id"].asString();
            task.start_time = json_task["start_time"].asDouble();
            for (auto robot_id : json_task["team_robot_ids"])
            {
                std::string robot_id_str = robot_id.asString();
                task.team_robot_ids.push_back(robot_id_str);
            }

            // list of robots (each item in this list has a key (robot id) and value (list of actions)
            Json::Value &robot_action_list = json_task["robot_actions"];
            for (int i = 0; i < robot_action_list.size(); i++)
            {
                Json::Value::iterator it = robot_action_list[i].begin();
                std::string robot_id = it.key().asString();

                task.robot_actions[robot_id] = std::vector<Action>();
                const Json::Value &jactions = robot_action_list[i][robot_id];
                // iterate through all actions for this robot
                for (int j = 0; j < jactions.size(); j++)
                {
                    Action action = Action::fromJson(jactions[j]);
                    task.robot_actions[robot_id].push_back(action);
                }
            }
            return task;
        }
    };

    struct TaskStatus
    {
        std::string task_id;
        std::map<std::string, Waypoint> robot_waypoints;
        std::map<std::string, Action> current_robot_actions;
        std::map<std::string, std::vector<Action>> completed_robot_actions;
        std::map<std::string, int> robot_floor;
        std::map<std::string, float> task_duration;
        std::map<std::string, float> mission_duration;
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
