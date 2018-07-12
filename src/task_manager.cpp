#include "task_manager.hpp"
#include <iostream>

namespace ccu
{
    TaskManager::TaskManager(const ConfigParams& config_params)
        : ZyreBaseCommunicator(config_params.task_manager_zyre_params.nodeName,
                               config_params.task_manager_zyre_params.groups,
                               config_params.task_manager_zyre_params.messageTypes,
                               false),
          resource_manager(config_params),
          ccu_store(config_params.ropod_task_data_db_name) { }

    /**
     * Loads any existing task data (ongoing tasks, scheduled tasks) from the CCU store database
     */
    void TaskManager::restoreTaskData()
    {
        this->scheduled_tasks = this->ccu_store.getScheduledTasks();
        this->ongoing_task_ids = this->ccu_store.getOngoingTasks();
    }

    /**
    * Processes a task request message; ignores all other messages.
    * Only responds to messages of type TASK
    *
    * @param msgContent a ZyreMsgContent pointer
    */
    void TaskManager::recvMsgCallback(ZyreMsgContent* msgContent)
    {
        Json::Value json_msg = this->convertZyreMsgToJson(msgContent);

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
            double task_start_time = json_msg["payload"]["startTime"].asFloat();

            TaskRequest task_request;
            task_request.user_id = user_id;
            task_request.cart_type = device_type;
            task_request.cart_id = device_id;
            task_request.start_time = task_start_time;
            task_request.pickup_pose.semantic_id = pickup_location;
            task_request.delivery_pose.semantic_id = delivery_location;
            this->processTaskRequest(task_request);
        }
        else if (message_type == "TASK_PROGRESS")
        {
            std::string task_id = json_msg["payload"]["taskId"].asString();
            std::string robot_id = json_msg["payload"]["robotId"].asString();
            std::string current_action = json_msg["payload"]["status"]["currentAction"].asString();
            std::string task_status = json_msg["payload"]["status"]["taskStatus"].asString();
            this->updateTaskStatus(task_id, robot_id, current_action, task_status);
        }
    }

    /**
     * Processes a task request, namely chooses robots for the task
     * and generates an appropriate task plan
     *
     * @param request a const reference to a TaskRequest object representing a task request
     */
    void TaskManager::processTaskRequest(const TaskRequest& request)
    {
        std::vector<Action> task_plan = this->task_planner.getTaskPlan(request);
        std::vector<Action> expanded_task_plan = this->path_planner.expandTaskPlan(task_plan);
        for (int i=0; i<expanded_task_plan.size(); i++)
        {
            expanded_task_plan[i].id = this->generateUUID();
        }

        std::vector<std::string> task_robots = this->resource_manager.getRobotsForTask(request, task_plan);
        Task task;

        task.id = this->generateUUID();
        task.start_time = request.start_time;
        task.team_robot_ids = task_robots;
        for (std::string robot_id : task_robots)
        {
            task.robot_actions[robot_id] = expanded_task_plan;
        }

        this->scheduled_tasks[task.id] = task;
        this->ccu_store.addTask(task);
    }

    /**
     * Dispatches all scheduled tasks that are ready for dispatching
     */
    void TaskManager::dispatchTasks()
    {
        for (auto task : this->scheduled_tasks)
        {
            std::string task_id = task.first;
            if (std::find(this->ongoing_task_ids.begin(), this->ongoing_task_ids.end(), task_id) == this->ongoing_task_ids.end())
            {
                bool is_task_executable = canExecuteTask(task_id);
                if (is_task_executable)
                {
                    this->dispatchTask(task.second);
                    this->ongoing_task_ids.push_back(task_id);
                    this->ccu_store.addOngoingTask(task_id);
                    this->initialiseTaskStatus(task_id);
                }
            }
        }
    }

    /**
     * Returns true if the given task needs to be dispatched
     * based on the task schedule; returns false otherwise
     *
     * @param task_id UUID representing the ID of a task
     */
    bool TaskManager::canExecuteTask(std::string task_id)
    {
        auto now = std::chrono::high_resolution_clock::now();
        double current_time = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() / 1000.0;
        double task_start_time = this->scheduled_tasks[task_id].start_time;
        if (task_start_time < current_time)
            return true;
        return false;
    }

    /**
     * Sends a task to the appropriate robot fleet
     *
     * @param task a const reference to a Task object representing a task
     */
    void TaskManager::dispatchTask(const Task& task)
    {
        for (auto actions_per_robot : task.robot_actions)
        {
            std::string current_robot_id = actions_per_robot.first;
            std::vector<Action> actions = actions_per_robot.second;

            Json::Value json_msg;
            json_msg["type"] = "TASK";
            json_msg["metamodel"] = "ropod-msg-schema.json";
            json_msg["msgId"] = this->generateUUID();
            json_msg["robotId"] = current_robot_id;

            char * timestr = zclock_timestr();
            json_msg["timestamp"] = timestr;
            zstr_free(&timestr);

            json_msg["payload"]["metamodel"] = "ropod-task-schema.json";
            json_msg["payload"]["taskId"] = task.id;

            Json::Value &action_list = json_msg["payload"]["actions"];
            for (Action action : actions)
            {
                Json::Value action_json = action.toJson();
                action_list.append(action_json);
            }

            Json::Value &robot_list = json_msg["payload"]["teamRobotIds"];
            for (std::string robot_id : task.team_robot_ids)
            {
                robot_list.append(robot_id);
            }

            std::string msg = Json::writeString(this->json_stream_builder, json_msg);
            this->shout(msg);
        }
    }

    /**
     * Creates a 'TaskStatus' entry in 'this->task_statuses' for the task with ID 'task_id'.
     *
     * @param task_id UUID representing the ID of a task
     */
    void TaskManager::initialiseTaskStatus(std::string task_id)
    {
        Task task = this->scheduled_tasks[task_id];

        TaskStatus task_status;
        task_status.task_id = task_id;
        for (std::string robot_id : task.team_robot_ids)
        {
            task_status.current_robot_action[robot_id] = task.robot_actions[robot_id][0].id;
            task_status.estimated_task_duration = task.estimated_duration;
        }
        this->task_statuses[task_id] = task_status;
    }

    /**
     * Updates the status of the robot with ID 'robot_id' that is performing
     * the task with ID 'task_id'.
     *
     * If 'task_status' is "terminated" or "completed", removes the task from the list
     * of scheduled and ongoing tasks and saves a historical database entry for the task.
     * On the other hand, if 'task_status' is "ongoing", updates the task entry
     * for the appropriate robot.
     *
     * @param task_id UUID representing a previously scheduled task
     * @param robot_id name of a robot
     * @param current_action UUID representing an action
     * @param task_status a string representing the status of a task;
     *        takes the values "ongoing", "terminated", and "completed"
     */
    void TaskManager::updateTaskStatus(std::string task_id, std::string robot_id,
                                       std::string current_action, std::string task_status)
    {
        TaskStatus status = this->task_statuses[task_id];
        status.status = task_status;
        if ((task_status == "terminated") || (task_status == "completed"))
        {
            Task task = this->scheduled_tasks[task_id];
            this->ccu_store.archiveTask(task, status);

            this->scheduled_tasks.erase(task_id);
            this->task_statuses.erase(task_id);

            auto task_pos = std::find(this->ongoing_task_ids.begin(), this->ongoing_task_ids.end(), task_id);
            if (task_pos != this->ongoing_task_ids.end())
                this->ongoing_task_ids.erase(task_pos);
        }
        else if (task_status == "ongoing")
        {
            std::string previous_action = status.current_robot_action[robot_id];
            status.completed_robot_actions[robot_id].push_back(previous_action);
            status.current_robot_action[robot_id] = current_action;

            //TODO: update the estimated time duration based on the current timestamp
            //and the estimated duration of the rest of the tasks
        }
    }

    /**
     * Returns the action with ID 'action_id' that is part of the task with ID 'task_id'
     * and is performed by the robot with ID 'robot_id'
     *
     * @param task_id UUID representing a scheduled task
     * @param robot_id name of a robot
     * @param action_id UUID representing a task
     */
    Action TaskManager::getAction(std::string task_id, std::string robot_id, std::string action_id)
    {
        Task task = this->scheduled_tasks[task_id];
        Action desired_action;
        for (Action action : task.robot_actions[robot_id])
        {
            if (action.id == action_id)
            {
                desired_action = action;
                break;
            }
        }
        return desired_action;
    }
}
