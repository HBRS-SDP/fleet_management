#include "db/ccu_store.hpp"
#include <sstream>

CCUStore::CCUStore(std::string db_name)
    : db_instance{}
{
    this->db_name = db_name;
}

/**
 * Saves the given task to a database as a new document under the "tasks" collection
 *
 * @param task a reference to a const ccu::Task object representing a task
 */
void CCUStore::addTask(const ccu::Task& task)
{
    mongocxx::client db_client{mongocxx::uri{}};
    auto collection = db_client[this->db_name]["tasks"];
    bsoncxx::builder::stream::document document{};
    //TODO: save the current timestamp
    Json::Value task_json = task.toJson();
    std::string task_string = Json::writeString(this->json_stream_builder, task_json);
    bsoncxx::document::value value = bsoncxx::from_json(task_string);
    collection.insert_one(value.view());
}

/**
 * Saves the given task id to a database as a new document under the "ongoing_tasks" collection
 *
 * @param task_id an integer representing the id of an already scheduled task
 */
void CCUStore::addOngoingTask(std::string task_id)
{
    mongocxx::client db_client{mongocxx::uri{}};
    auto collection = db_client[this->db_name]["ongoing_tasks"];
    bsoncxx::builder::stream::document document{};
    //TODO: save the current timestamp
    document << "task_id" << task_id;
    collection.insert_one(document.view());
}

/**
 * Returns a vector of ids representing all tasks that are saved
 * under the "ongoing_tasks" collection
 */
std::vector<std::string> CCUStore::getOngoingTasks()
{
    mongocxx::client db_client{mongocxx::uri{}};
    auto database = db_client[this->db_name];
    auto collection = database["ongoing_tasks"];
    auto cursor = collection.find({});

    std::vector<std::string> task_ids;
    for (auto doc : cursor)
    {
        std::string task_id = doc["task_id"].get_utf8().value.to_string();
        task_ids.push_back(task_id);
    }
    return task_ids;
}

/**
 * Returns a dictionary of task IDs and ccu::Task objects representing
 * the scheduled tasks that are saved under the "tasks" collection
 */
std::map<std::string, ccu::Task> CCUStore::getScheduledTasks()
{
    mongocxx::client db_client{mongocxx::uri{}};
    auto database = db_client[this->db_name];
    auto collection = database["tasks"];
    auto cursor = collection.find({});

    std::map<std::string, ccu::Task> scheduled_tasks;
    for (auto doc : cursor)
    {
        std::string task_id = doc["id"].get_utf8().value.to_string();
        ccu::Task task = this->getTask(task_id);
        scheduled_tasks[task_id] = task;
    }
    return scheduled_tasks;
}

/**
 * Returns a ccu::Task object representing the task with the given id
 *
 * @param task_id an integer representing the id of a task
 */
ccu::Task CCUStore::getTask(std::string task_id)
{
    mongocxx::client db_client{mongocxx::uri{}};
    auto database = db_client[this->db_name];
    auto collection = database["tasks"];
    auto doc = collection.find_one(bsoncxx::builder::stream::document{}
                                   << "id" << task_id
                                   << bsoncxx::builder::stream::finalize);
    std::string json_doc = bsoncxx::to_json((*doc));
    ccu::Task task = ccu::Task::fromJson(json_doc);
    return task;
}
