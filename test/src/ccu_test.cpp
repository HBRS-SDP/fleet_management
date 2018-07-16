#include <chrono>
#include <thread>

#include "data_structures/action.hpp"
#include "data_structures/task.hpp"
#include "data_structures/task_status.hpp"
#include "task_manager.hpp"
#include "config/config_params.hpp"
#include "config/config_file_reader.hpp"

int main()
{
    ConfigParams config_params = ConfigFileReader::load("../../config/ccu_config.yaml");
    ccu::TaskManager task_manager(config_params);

    task_manager.restoreTaskData();
    std::map<std::string, ccu::Task> scheduled_tasks = task_manager.getScheduledTasks();
    std::cout << "Scheduled tasks " << std::endl;
    for (auto scheduled_task_data : scheduled_tasks)
    {
        ccu::Task task = scheduled_task_data.second;

        std::cout << "id: " << task.id << std::endl;
        std::cout << std::fixed << "start_time: " << task.start_time << std::endl;
        std::cout << "robots: " << std::endl;
        for (std::string robot_id : task.team_robot_ids)
        {
            std::cout << robot_id;
            std::vector<ccu::Action> task_plan = task.robot_actions[robot_id];
            std::cout << " num actions: " << task_plan.size() << std::endl;
        }
        std::cout << std::endl;
    }
    std::cout << "-------" << std::endl;

    std::vector<std::string> ongoing_tasks = task_manager.getOngoingTasksIds();
    std::cout << "ongoing task ids: [";
    for (std::string task_id : ongoing_tasks)
    {
        std::cout << task_id;
    }
    std::cout << "] " << std::endl;
    std::cout << "-------" << std::endl;

    std::map<std::string, ccu::TaskStatus> task_statuses = task_manager.getOngoingTaskStatuses();
    std::cout << "Ongoing task statuses " << std::endl;
    for (auto task_status_data : task_statuses)
    {
        ccu::TaskStatus task_status = task_status_data.second;

        std::cout << "task_id: " << task_status.task_id << std::endl;
        std::cout << "status: " << task_status.status << std::endl;
        std::cout << "current robot actions:" << std::endl;
        for (auto robot_action_data : task_status.current_robot_action)
        {
            std::string robot_id = robot_action_data.first;
            std::string action_id = robot_action_data.second;
            std::cout << robot_id << " " << action_id << std::endl;
        }

        std::cout << "completed robot actions:" << std::endl;
        for (auto robot_action_data : task_status.completed_robot_actions)
        {
            std::string robot_id = robot_action_data.first;
            std::vector<std::string> completed_robot_actions = robot_action_data.second;
            std::cout << robot_id << " [ ";
            for (std::string action_id : completed_robot_actions)
            {
                std::cout << action_id << " ";
            }
            std::cout << "]" << std::endl;
        }
        std::cout << std::endl;
    }

    return 0;
}
