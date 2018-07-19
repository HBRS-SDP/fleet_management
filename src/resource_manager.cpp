#include <iostream>
#include "resource_manager.hpp"

namespace ccu
{
    ResourceManager::ResourceManager(const ConfigParams& config_params)
    : ZyreBaseCommunicator(config_params.resource_manager_zyre_params.nodeName,
                            config_params.resource_manager_zyre_params.groups,
                            config_params.resource_manager_zyre_params.messageTypes,
                            false)
    {
        robot_ids = config_params.ropod_ids;
        elevator_ids = config_params.elevator_ids;
    }

    std::vector<std::string> ResourceManager::getRobotsForTask(const TaskRequest& task_request,
                                                               const std::vector<Action>& task_plan)
   {
       std::vector<std::string> task_robots;
       task_robots.push_back("ropod_0");
       return task_robots;
   }

    /**
     * Process an elevator request from a ropod.
     *
     */

    void ResourceManager::recvMsgCallback(ZyreMsgContent* msgContent)
    {
        Json::Value json_msg = this->convertZyreMsgToJson(msgContent);

        if (json_msg == Json::nullValue)
            return;

        std::string message_type = json_msg["header"]["type"].asString();
        if (message_type == "ELEVATOR-CMD")
        {
            std::string command = json_msg["payload"]["command"].asString();
            std::string query_id = json_msg["payload"]["queryId"].asString();

            if (command == "CALL_ELEVATOR")
            {
                int start_floor = json_msg["payload"]["startFloor"].asInt();
                int goal_floor = json_msg["payload"]["goalFloor"].asInt();
                std::string task_id = json_msg["payload"]["taskId"].asString();
                std::string load = json_msg["payload"]["load"].asString();

                std::cout << "[INFO] Received elevator request from ropod";

                ElevatorRequest robot_request;

                // TODO: Choose elevator
                robot_request.elevator_id = 1;
                robot_request.operational_mode = "ROBOT";

                // TODO: Store this query somewhere

                robot_request.query_id = query_id;
                robot_request.command = command;
                robot_request.start_floor = start_floor;
                robot_request.goal_floor = goal_floor;
                robot_request.task_id = task_id;
                robot_request.load = load;
                robot_request.robot_id = "ropod_0";
                robot_request.status = "pending";

                requestElevator(start_floor, goal_floor, robot_request.elevator_id, robot_request.query_id);


            }
            else if (command == "CANCEL_CALL")
            {
                //int start_floor = json_msg["payload"]["startFloor"].asInt();
                //int goal_floor = json_msg["payload"]["goalFloor"].asInt();

            }
        }
        else if (message_type == "ELEVATOR-CMD-REPLY")
        {
            std::string query_id = json_msg["payload"]["queryId"].asString();
            bool query_success = json_msg["payload"]["querySuccess"].asBool();

            std::cout << "[INFO] Received reply from elevator control";
        }
        else if (message_type == "ROBOT-CALL-UPDATE")
        {
            std::string command = json_msg["payload"]["command"].asString();
            std::string query_id = json_msg["payload"]["queryId"].asString();
            if (command == "ROBOT_FINISHED_ENTERING")
            {
                // Close the doors

                std::cout << "[INFO] Received entering confirmation from ropod";

            }
            else if (command == "ROBOT_FINISHED_EXITING")
            {
                // Close the doors
                std::cout << "[INFO] Received exiting confirmation from ropod";


            }
        }
        else if (message_type == "ROBOT-CALL-UPDATE-REPLY")
        {
            std::string query_id = json_msg["payload"]["queryId"].asString();
            std::cout << "[INFO] Received exiting confirmation from ropod";
        }
    }

    void ResourceManager::requestElevator(int startFloor, int goalFloor, int elevatorId, std::string query_id)
    {
        Json::Value root;
        root["header"]["type"] = "ELEVATOR-CMD";
        root["header"]["metamodel"] = "ropod-msg-schema.json";
        root["header"]["msgId"] = generateUUID();
        root["header"]["timestamp"] = "2018-07-17T12:27:53Z";
        root["header"]["timestamp"] = getTimeStamp();

        root["payload"]["metamodel"] = "ropod-elevator-cmd-schema.json";
        root["payload"]["startFloor"] = startFloor;
        root["payload"]["goalFloor"] = goalFloor;
        root["payload"]["elevatorId"] = elevatorId;
        root["payload"]["operationalMode"] = "ROBOT";
        root["payload"]["queryId"] = query_id;

        std::string msg = convertJsonToString(root);
        shout(msg, "ELEVATOR-CONTROL");
    }

    RobotStatus ResourceManager::getRobotStatus(const std::string& robot_id)
    {
        return this->robot_statuses[robot_id];
    }


}
