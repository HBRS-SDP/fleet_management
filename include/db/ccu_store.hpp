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

    void addTask(const task::Task& task);
    void addOngoingTask(int task_id);
    std::vector<int> getOngoingTasks();
    task::Task getTask(int task_id);
    std::vector<task::Task> getScheduledTasks();
private:
    std::string db_name_;
    mongocxx::instance db_instance_;
    Json::StreamWriterBuilder json_stream_builder_;
};

#endif
