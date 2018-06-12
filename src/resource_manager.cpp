#include "resource_manager.hpp"

namespace task
{
    ResourceManager::ResourceManager(const ConfigParams& config_params)
    {
        robot_ids_ = config_params.ropod_ids;
        elevator_ids_ = config_params.elevator_ids;
    }

    std::vector<std::string> ResourceManager::getRobotsForTask(const TaskRequest& task_request,
                                                               const std::vector<Action>& task_plan)
   {
       std::vector<std::string> task_robots;
       task_robots.push_back("ropod_0");
       return task_robots;
   }
}
