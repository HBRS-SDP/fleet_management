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
     * @param task a reference to a const ccu::Task object representing a task
     */
    void addTask(const ccu::Task& task);

    /**
     * Saves the given task to a database as a new document under the "task_archive" collection
     *
     * @param task a previously scheduled task
     * @param task_status status task status description
     */
    void archiveTask(ccu::Task task, ccu::TaskStatus task_status);

    /**
     * Saves the given task id to a database as a new document under the "ongoing_tasks" collection
     *
     * @param task_id UUID representing the id of an already scheduled task
     */
    void addOngoingTask(std::string task_id);

    /**
     * Returns a vector of ids representing all tasks that are saved
     * under the "ongoing_tasks" collection
     */
    std::vector<std::string> getOngoingTasks();

    /**
     * Returns a dictionary of task IDs and ccu::Task objects representing
     * the scheduled tasks that are saved under the "tasks" collection
     */
    std::map<std::string, ccu::Task> getScheduledTasks();

    /**
     * Returns a ccu::Task object representing the task with the given id
     *
     * @param task_id UUID representing the id of a task
     */
    ccu::Task getTask(std::string task_id);

private:
    std::string db_name;
    mongocxx::instance db_instance;
    Json::StreamWriterBuilder json_stream_builder;
};

#endif
