#ifndef TASK_MANAGER_HPP
#define TASK_MANAGER_HPP

#include <sstream>
#include <vector>
#include <algorithm>
#include <json/json.h>

#include "config/config_params.hpp"
#include "data_structures/task.hpp"
#include "task_planner.hpp"
#include "path_planner.hpp"
#include "task_executor.hpp"
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

        void processTaskRequest(const TaskRequest& request);
        void dispatchTasks();
    private:
        /**
         * Converts msg_params->message to a json message
         *
         * @param msg_params message data
         */
        Json::Value convertZyreMsgToJson(ZyreMsgContent* msg_params);

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
