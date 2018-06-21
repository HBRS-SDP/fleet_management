#include "task_manager.hpp"

namespace task
{
    TaskManager::TaskManager(const ConfigParams& config_params)
        : ZyreBaseCommunicator(config_params.task_manager_zyre_params.nodeName,
                               config_params.task_manager_zyre_params.groups,
                               config_params.task_manager_zyre_params.messageTypes,
                               false),
          resource_manager_(config_params),
          ccu_store_(config_params.ropod_task_data_db_name) { }

    void TaskManager::recvMsgCallback(ZyreMsgContent* msgContent)
    {
        Json::Value json_msg = convertZyreMsgToJson(msgContent);

        if (json_msg == Json::nullValue)
            return;

        std::string message_type = json_msg["header"]["type"].asString();
        if (message_type == "TASK")
        {
            std::string user_id = json_msg["payload"]["userId"].asString();
            std::string device_type = json_msg["payload"]["deviceType"].asString();
            std::string device_id = json_msg["payload"]["deviceId"].asString();
            std::string pickup_location = json_msg["payload"]["pickupLocation"].asString();
            std::string delivery_location = json_msg["payload"]["deliveryLocation"].asString();
            float task_start_time = json_msg["payload"]["startTime"].asFloat();
        }
    }

    /**
     * Converts msg_params.message to a json message
     *
     * @param msg_params message data
     */
    Json::Value TaskManager::convertZyreMsgToJson(ZyreMsgContent* msg_params)
    {
        if (msg_params->event == "SHOUT")
        {
            std::stringstream msg_stream;
            msg_stream << msg_params->message;

            Json::Value root;
            Json::CharReaderBuilder reader_builder;
            std::string errors;
            bool ok = Json::parseFromStream(reader_builder, msg_stream, &root, &errors);

            return root;
        }

        return Json::nullValue;
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
        ccu_store_.addTask(task);
    }

    void TaskManager::dispatchTasks()
    {
        for (auto task : scheduled_tasks_)
        {
            int task_id = task.first;
            if (std::find(ongoing_task_ids_.begin(), ongoing_task_ids_.end(), task_id) == ongoing_task_ids_.end())
            {
                bool is_task_executable = canExecuteTask(task_id);
                if (is_task_executable)
                {
                    dispatchTask(task.second);
                    ongoing_task_ids_.push_back(task_id);
                    ccu_store_.addOngoingTask(task_id);
                }
            }
        }
    }

    bool TaskManager::canExecuteTask(int task_id)
    {
        float current_time = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count();
        float task_start_time = scheduled_tasks_[task_id].start_time;
        if (task_start_time > current_time)
            return true;
        return false;
    }

    bool TaskManager::dispatchTask(const Task& task)
    {
        return true;
    }
}
