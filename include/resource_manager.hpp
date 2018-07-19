#ifndef RESOURCE_MANAGER_HPP
#define RESOURCE_MANAGER_HPP

#include <vector>
#include <map>

#include "config/config_params.hpp"
#include "data_structures/action.hpp"
#include "data_structures/task_request.hpp"
#include "data_structures/task.hpp"
#include "data_structures/robot_task.hpp"
#include "data_structures/robot_status.hpp"
#include "data_structures/elevator_requests.hpp"
#include "ZyreBaseCommunicator.h"


namespace ccu
{
    class ResourceManager : ZyreBaseCommunicator
    {
    public:
        ResourceManager(const ConfigParams& config_params);
        ~ResourceManager() { }

        std::vector<std::string> getRobotsForTask(const TaskRequest& task_request,
                                                  const std::vector<Action>& task_plan);

        virtual void recvMsgCallback(ZyreMsgContent* msgContent);
        RobotStatus getRobotStatus(const std::string& robot_id);
        void requestElevator(int startFloor, int goalFloor, int elevatorId, std::string query_id);
        void confirmRobotAction(const std::string robot_action, const std::string robot_id);
        void confirmElevator(const std::string query_id);


    private:
        std::vector<std::string> robot_ids;
        std::vector<int> elevator_ids;
        std::map<std::string, std::vector<RobotTask>> scheduled_robot_tasks;
        std::map<int, ElevatorRequests> elevator_requests;
        std::map<std::string, RobotStatus> robot_statuses;


    };
}

#endif
