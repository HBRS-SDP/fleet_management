#include "db/ccu_store.hpp"

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
    document << task_string;
    collection.insert_one(document.view());
}

/**
 * Saves the given task id to a database as a new document under the "ongoing_tasks" collection
 *
 * @param task_id an integer representing the id of an already scheduled task
 */
void CCUStore::addOngoingTask(int task_id)
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
std::vector<int> CCUStore::getOngoingTasks()
{
    mongocxx::client db_client{mongocxx::uri{}};
    auto database = db_client[this->db_name];
    auto collection = database["ongoing_tasks"];
    auto cursor = collection.find({});

    std::vector<int> task_ids;
    for (auto doc : cursor)
    {
        int task_id = doc["task_id"].get_int32();
        task_ids.push_back(task_id);
    }
    return task_ids;
}

/**
 * Returns a ccu::Task object representing the task with the given id
 *
 * @param task_id an integer representing the id of a task
 */
ccu::Task CCUStore::getTask(int task_id)
{
    mongocxx::client db_client{mongocxx::uri{}};
    auto database = db_client[this->db_name];
    auto collection = database["tasks"];
    auto doc = collection.find_one(bsoncxx::builder::stream::document{}
                                   << "id" << task_id
                                   << bsoncxx::builder::stream::finalize);
    auto document_view = (*doc).view();

    ccu::Task task;
    task.id = document_view["id"].get_int32();

    return task;
}

/**
 * Returns a dictionary of task IDs and ccu::Task objects representing
 * the scheduled tasks that are saved under the "tasks" collection
 */
std::map<int, ccu::Task> CCUStore::getScheduledTasks()
{
    return std::map<int, ccu::Task>();
}
