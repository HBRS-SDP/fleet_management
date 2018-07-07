#include <chrono>

#include "data_structures/task.hpp"
#include "task_manager.hpp"
#include "config/config_params.hpp"
#include "config/config_file_reader.hpp"

void sendTask(ccu::TaskManager& task_manager)
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

int main()
{
    ConfigParams config_params = ConfigFileReader::load("../../config/ccu_config.yaml");
    ccu::TaskManager task_manager(config_params);

    sendTask(task_manager);
    return 0;
}
