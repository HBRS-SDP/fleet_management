#ifndef TASK_MANAGER_HPP
#define TASK_MANAGER_HPP

#include <vector>
#include <algorithm>
#include "config/config_params.hpp"
#include "data_structures/task.hpp"
#include "task_planner.hpp"
#include "path_planner.hpp"
#include "task_executor.hpp"
#include "resource_manager.hpp"

namespace task
{
    class TaskManager
    {
    public:
        TaskManager(const ConfigParams& config_params);
        ~TaskManager() { }

        void processTaskRequest(const TaskRequest& request);
        void monitorTasks();
    private:
        bool canExecuteTask(int task_id);

        std::map<int, Task> scheduled_tasks_;
        std::vector<int> ongoing_task_ids_;
        TaskPlanner task_planner_;
        PathPlanner path_planner_;
        TaskExecutor task_executor_;
        ResourceManager resource_manager_;
    };
}

#endif
