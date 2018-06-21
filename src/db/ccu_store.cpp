#include "db/ccu_store.hpp"

CCUStore::CCUStore(std::string db_name)
    : db_name_(db_name), db_instance_{}
{ }

void CCUStore::addTask(const task::Task& task)
{
    mongocxx::client db_client{mongocxx::uri{}};
    auto collection = db_client[db_name_]["tasks"];
    bsoncxx::builder::stream::document document{};
    //TODO: save the current timestamp
    Json::Value task_json = task.toJson();
    std::string task_string = Json::writeString(json_stream_builder_, task_json);
    document << task_string;
    collection.insert_one(document.view());
}

void CCUStore::addOngoingTask(int task_id)
{
    mongocxx::client db_client{mongocxx::uri{}};
    auto collection = db_client[db_name_]["ongoing_tasks"];
    bsoncxx::builder::stream::document document{};
    //TODO: save the current timestamp
    document << "task_id" << task_id;
    collection.insert_one(document.view());
}

std::vector<int> CCUStore::getOngoingTasks()
{
    return std::vector<int>();
}

task::Task CCUStore::getTask(int task_id)
{
    return task::Task();
}

std::vector<task::Task> CCUStore::getScheduledTasks()
{
    return std::vector<task::Task>();
}
