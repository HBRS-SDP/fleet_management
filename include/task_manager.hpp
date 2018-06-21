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

namespace task
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
        bool dispatchTask(const Task& task);
    private:
        /**
         * Converts msg_params->message to a json message
         *
         * @param msg_params message data
         */
        Json::Value convertZyreMsgToJson(ZyreMsgContent* msg_params);

        /**
         * Returns true if the given task needs to be dispatched
         * based on the task schedule; returns false otherwise
         *
         * @param task_id an integer representing the ID of a task
         */
        bool canExecuteTask(int task_id);

        std::map<int, Task> scheduled_tasks_;
        std::vector<int> ongoing_task_ids_;
        TaskPlanner task_planner_;
        PathPlanner path_planner_;
        ResourceManager resource_manager_;
        CCUStore ccu_store_;
    };
}

#endif
