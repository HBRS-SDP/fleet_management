#include "task_manager.hpp"

namespace task
{
    TaskManager::TaskManager(const ConfigParams& config_params)
        : resource_manager_(config_params) { }

    void TaskManager::monitorTasks()
    {
        for (auto task : scheduled_tasks_)
        {
            int task_id = task.first;
            if (std::find(ongoing_task_ids_.begin(), ongoing_task_ids_.end(), task_id) == ongoing_task_ids_.end())
            {
                bool is_task_executable = canExecuteTask(task_id);
                if (is_task_executable)
                {
                    task_executor_.executeTask(task.second);
                    ongoing_task_ids_.push_back(task_id);
                }
            }
            else
            {

            }
        }
    }

    void TaskManager::processTaskRequest(const TaskRequest& request)
    {
        std::vector<Action> task_plan = task_planner_.getTaskPlan(request);
        std::vector<Action> expanded_task_plan = path_planner_.expandTaskPlan(task_plan);
        std::vector<std::string> task_robots = resource_manager_.getRobotsForTask(request, task_plan);
        Task task;

        task.id = 0;
        task.start_time = request.start_time;
        task.team_robot_ids = task_robots;
        for (std::string robot_id : task_robots)
        {
            task.robot_actions[robot_id] = task_plan;
        }

        scheduled_tasks_[task.id] = task;
    }

    bool TaskManager::canExecuteTask(int task_id)
    {
        float current_time = 0.0f; //TODO: get the current timestamp here
        float task_start_time = scheduled_tasks_[task_id].start_time;
        if (task_start_time > current_time)
            return true;
        return false;
    }
}
