#ifndef TASK_STATUS_HPP
#define TASK_STATUS_HPP

#include <string>
#include <vector>
#include <map>
#include <json/json.h>

namespace ccu
{
    struct TaskStatus
    {
        std::string task_id;
        std::string status;
        std::map<std::string, std::string> current_robot_action;
        std::map<std::string, std::vector<std::string>> completed_robot_actions;
        double estimated_task_duration;

        Json::Value toJson() const
        {
            Json::Value task_json;
            task_json["task_id"] = task_id;
            task_json["status"] = status;

            Json::Value &robot_action_list = task_json["current_robot_actions"];
            for (auto actions_per_robot : current_robot_action)
            {
                std::string robot_id = actions_per_robot.first;
                std::string action = actions_per_robot.second;

                Json::Value current_robot_action_json;
                current_robot_action_json[robot_id] = action;
                robot_action_list.append(current_robot_action_json);
            }

            Json::Value &completed_robot_action_list = task_json["completed_robot_actions"];
            for (auto actions_per_robot : completed_robot_actions)
            {
                std::string robot_id = actions_per_robot.first;
                std::vector<std::string> actions = actions_per_robot.second;

                Json::Value action_list;
                for (std::string action : actions)
                {
                    action_list.append(action);
                }

                Json::Value robot_action_list_json;
                robot_action_list_json[robot_id] = action_list;
                robot_action_list.append(robot_action_list_json);
            }

            return task_json;
        }

        static TaskStatus fromJson(const std::string &json_string)
        {
            Json::Value json_task_status;

            Json::CharReaderBuilder json_builder;
            Json::CharReader* json_reader = json_builder.newCharReader();
            std::string errors;
            bool parsingSuccessful = json_reader->parse(json_string.c_str(), json_string.c_str() + json_string.size(), &json_task_status, &errors);
            delete json_reader;

            TaskStatus task_status;
            task_status.task_id = json_task_status["task_id"].asString();
            task_status.status = json_task_status["status"].asString();

            Json::Value &robot_action_list = json_task_status["current_robot_actions"];
            for (int i = 0; i < robot_action_list.size(); i++)
            {
                Json::Value::iterator it = robot_action_list[i].begin();
                std::string robot_id = it.key().asString();
                std::string current_action = robot_action_list[i][robot_id].asString();
                task_status.current_robot_action[robot_id] = current_action;
            }

            Json::Value &completed_robot_action_list = json_task_status["completed_robot_actions"];
            for (int i=0; i<robot_action_list.size(); i++)
            {
                Json::Value::iterator it = completed_robot_action_list[i].begin();
                std::string robot_id = it.key().asString();

                task_status.completed_robot_actions[robot_id] = std::vector<std::string>();
                const Json::Value &actions = completed_robot_action_list[i][robot_id];
                for (auto action : actions)
                {
                    std::string action_id = action.asString();
                    task_status.completed_robot_actions[robot_id].push_back(action_id);
                }
            }
            return task_status;
        }
    };
};

#endif