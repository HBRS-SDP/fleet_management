#ifndef TASK_MANAGER_HPP
#define TASK_MANAGER_HPP

#include <chrono>
#include <sstream>
#include <vector>
#include <algorithm>
#include <json/json.h>

#include "config/config_params.hpp"
#include "data_structures/action.hpp"
#include "data_structures/task_request.hpp"
#include "data_structures/task.hpp"
#include "data_structures/task_status.hpp"
#include "db/ccu_store.hpp"
#include "task_planner.hpp"
#include "path_planner.hpp"
#include "resource_manager.hpp"
#include "ZyreBaseCommunicator.h"

namespace ccu
{
    /**
     * An interface for handling ropod task requests and managing ropod tasks
     *
     * @author Alex Mitrevski
     * @maintainer Alex Mitrevski, Argentina Ortega Sainz
     * @contact aleksandar.mitrevski@h-brs.de, argentina.ortega@h-brs.de
     */
    class TaskManager : ZyreBaseCommunicator
    {
    public:
        TaskManager(const ConfigParams& config_params);
        ~TaskManager() { }

        /**
         * Loads any existing task data (ongoing tasks, scheduled tasks) from the CCU store database
         */
        void restoreTaskData();

        /**
        * Processes a task request message; ignores all other messages.
        * Only responds to messages of type TASK-REQUEST and TASK-PROGRESS
        *
        * @param msgContent a ZyreMsgContent pointer
        */
        virtual void recvMsgCallback(ZyreMsgContent* msgContent);

        /**
         * Processes a task request, namely chooses robots for the task
         * and generates an appropriate task plan
         *
         * @param request a const reference to a TaskRequest object representing a task request
         */
        void processTaskRequest(const TaskRequest& request);

        /**
         * Dispatches all scheduled tasks that are ready for dispatching
         */
        void dispatchTasks();

        /**
         * Sends a task to the appropriate robot fleet
         *
         * @param task a const reference to a Task object representing a task
         */
        void dispatchTask(const Task& task);

        /**
         * Returns the scheduled tasks
         */
        std::map<std::string, Task> getScheduledTasks() const;

        /**
         * Returns the task IDs of ongoing tasks
         */
        std::vector<std::string> getOngoingTasksIds() const;

        /**
         * Returns the statuses of the ongoing tasks
         */
        std::map<std::string, TaskStatus> getOngoingTaskStatuses() const;
    private:
        std::vector<Action> expandTaskPlan(const std::vector<Action>& task_plan);

        /**
         * Returns true if the given task needs to be dispatched
         * based on the task schedule; returns false otherwise
         *
         * @param task_id UUID representing the ID of a task
         */
        bool canExecuteTask(std::string task_id);

        /**
         * Creates a 'TaskStatus' entry in 'this->task_statuses' for the task with ID 'task_id'.
         *
         * @param task_id UUID representing the ID of a task
         */
        void initialiseTaskStatus(std::string task_id);

        /**
         * Updates the status of the robot with ID 'robot_id' that is performing
         * the task with ID 'task_id'.
         *
         * If 'task_status' is "terminated", removes the task from the list of scheduled
         * and ongoing tasks and saves a historical database entry for the task.
         * On the other hand, if 'task_status' is "ongoing", the task's entry
         * is updated for the appropriate robot.
         *
         * @param task_id UUID representing a previously scheduled task
         * @param robot_id name of a robot
         * @param current_action UUID representing an action
         * @param task_status a string representing the status of a task;
         *        takes the values "ongoing", "terminated", and "completed"
         */
        void updateTaskStatus(std::string task_id, std::string robot_id,
                              std::string current_action, std::string task_status);

        /**
         * Returns the action with ID 'action_id' that is part of the task with ID 'task_id'
         * and is performed by the robot with ID 'robot_id'
         *
         * @param task_id UUID representing a scheduled task
         * @param robot_id name of a robot
         * @param action_id UUID representing a task
         */
        Action getAction(std::string task_id, std::string robot_id, std::string action_id);

        /**
         * Returns the current UNIX timestamp in seconds
         */
        double getCurrentTime() const;

        std::map<std::string, Task> scheduled_tasks;
        std::vector<std::string> ongoing_task_ids;
        std::map<std::string, TaskStatus> task_statuses;
        TaskPlanner task_planner;
        PathPlanner path_planner;
        ResourceManager resource_manager;
        CCUStore ccu_store;
        Json::StreamWriterBuilder json_stream_builder;
    };
}

#endif
