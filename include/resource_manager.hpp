#ifndef RESOURCE_MANAGER_HPP
#define RESOURCE_MANAGER_HPP

#include <vector>
#include <map>
#include "config/config_params.hpp"
#include "data_structures/task.hpp"

namespace task
{
    class ResourceManager
    {
    public:
        ResourceManager(const ConfigParams& config_params);
        ~ResourceManager() { }

        std::vector<std::string> getRobotsForTask(const TaskRequest& task_request,
                                                  const std::vector<Action>& task_plan);
    private:
        std::vector<std::string> robot_ids_;
        std::vector<int> elevator_ids_;
        std::map<std::string, std::vector<RobotTask>> scheduled_robot_tasks_;
        std::map<int, ElevatorRequests> elevator_requests_;
        std::map<std::string, RobotStatus> robot_statuses_;
    };
}

#endif
