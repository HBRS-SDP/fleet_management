#ifndef TASK_HPP
#define TASK_HPP

#include <string>
#include <vector>
#include <map>
#include <json/json.h>

#include "data_structures/action.hpp"

namespace ccu
{
    struct Task
    {
        std::string id;
        std::map<std::string, std::vector<Action>> robot_actions; //TODO: consider a different data structure for the list of actions
                                                                  //so that it's easier to expand it if necessary
        std::vector<std::string> team_robot_ids;
        double start_time;
        double estimated_duration;

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
}

#endif
