#include <chrono>
#include <thread>

#include "data_structures/task.hpp"
#include "task_manager.hpp"
#include "config/config_params.hpp"
#include "config/config_file_reader.hpp"

void dispatchTask(ccu::TaskManager& task_manager)
{
    ccu::Task task;
    task.id = 1;
    task.start_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();

    task.team_robot_ids = { "ropod_0" };

    ccu::Action go_to_action;
    go_to_action.id = "go_to_test";
    go_to_action.eta = -1.0f;
    go_to_action.execution_status = "in_progress";
    task.robot_actions["ropod_0"].push_back(go_to_action);

    task_manager.dispatchTask(task);
}

void sendTaskRequest(ccu::TaskManager& task_manager)
{
        ccu::TaskRequest task_request;
        task_request.user_id = "ccu_test";
        task_request.cart_type = "MobiDik";
        task_request.cart_id = "XYZ";
        auto now = std::chrono::system_clock::now();
        double current_time = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() / 1000.0;
        task_request.start_time = current_time;
        task_request.pickup_pose.semantic_id = "basement";
        task_request.delivery_pose.semantic_id = "ward";
        task_manager.processTaskRequest(task_request);
}

int main()
{
    ConfigParams config_params = ConfigFileReader::load("../../config/ccu_config.yaml");
    ccu::TaskManager task_manager(config_params);
    // test dispatchTask
    // listen to messages on the ROPOD group and check they are as expected
    dispatchTask(task_manager);

    sendTaskRequest(task_manager);
    task_manager.dispatchTasks();

    task_manager.restoreTaskData();
    std::map<std::string, ccu::Task> st = task_manager.getScheduledTasks();
    std::cout << "Scheduled tasks " << std::endl;
    for (std::map<std::string, ccu::Task>::iterator it=st.begin(); it!=st.end(); ++it)
    {
        std::cout << "id: " << it->second.id << std::endl;
        std::cout << std::fixed << "start_time: " << it->second.start_time << std::endl;
        std::cout << "robots: " << std::endl;
        for (std::string robot_id : it->second.team_robot_ids)
        {
            std::cout << robot_id;
            std::vector<ccu::Action> task_plan = it->second.robot_actions[robot_id];
            std::cout << " num actions: " << task_plan.size() << std::endl;
        }
        std::cout << "-------" << std::endl;
    }
    std::vector<std::string> ongoing_tasks = task_manager.getOngoingTasksIds();
    std::cout << "ongoing task ids: [";
    for (std::string task_id : ongoing_tasks)
    {
        std::cout << task_id;
    }
    std::cout << "] " << std::endl;

    return 0;
}
