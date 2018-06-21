#ifndef CCU_STORE_HPP
#define CCU_STORE_HPP

#include <string>

#include <bsoncxx/builder/stream/document.hpp>
#include <bsoncxx/json.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>
#include <json/json.h>

#include "data_structures/task.hpp"

/**
 * An interface for saving CCU data into and retrieving them from a database
 *
 * @author Alex Mitrevski
 * @contact aleksandar.mitrevski@h-brs.de
 */
class CCUStore
{
public:
    CCUStore(std::string db_name);
    ~CCUStore() { }

    /**
     * Saves the given task to a database as a new document under the "tasks" collection
     *
     * @param task a reference to a const task::Task object representing a task
     */
    void addTask(const task::Task& task);

    /**
     * Saves the given task id to a database as a new document under the "ongoing_tasks" collection
     *
     * @param task_id an integer representing the id of an already scheduled task
     */
    void addOngoingTask(int task_id);

    /**
     * Returns a vector of ids representing all tasks that are saved
     * under the "ongoing_tasks" collection
     */
    std::vector<int> getOngoingTasks();

    /**
     * Returns a task::Task object representing the task with the given id
     *
     * @param task_id an integer representing the id of a task
     */
    task::Task getTask(int task_id);

    /**
     * Returns a vector of task::Task object representing the scheduled tasks
     * that are saved under the "tasks" collection
     */
    std::vector<task::Task> getScheduledTasks();

private:
    std::string db_name_;
    mongocxx::instance db_instance_;
    Json::StreamWriterBuilder json_stream_builder_;
};

#endif
