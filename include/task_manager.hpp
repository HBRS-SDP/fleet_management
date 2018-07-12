#ifndef TASK_MANAGER_HPP
#define TASK_MANAGER_HPP

#include <chrono>
#include <sstream>
#include <vector>
#include <algorithm>
#include <json/json.h>

#include "config/config_params.hpp"
#include "data_structures/task.hpp"
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
         * Only responds to messages of type TASK
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
        std::map<int, Task> getScheduledTasks() {return scheduled_tasks;}

        /**
         * Returns the task IDs of ongoing tasks
         */
        std::vector<int> getOngoingTasksIds() {return ongoing_task_ids;}
    private:
        /**
         * Returns true if the given task needs to be dispatched
         * based on the task schedule; returns false otherwise
         *
         * @param task_id an integer representing the ID of a task
         */
        bool canExecuteTask(int task_id);

        std::map<int, Task> scheduled_tasks;
        std::vector<int> ongoing_task_ids;
        TaskPlanner task_planner;
        PathPlanner path_planner;
        ResourceManager resource_manager;
        CCUStore ccu_store;
        Json::StreamWriterBuilder json_stream_builder;
    };
}

#endif
