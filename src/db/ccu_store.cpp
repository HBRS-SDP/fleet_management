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
 * Saves the given task to a database as a new document under the "task_archive" collection
 * and deletes it from the "tasks", "ongoing_tasks", and "ongoing_task_status" collections
 *
 * @param task a previously scheduled task
 * @param task_status status task status description
 */
void CCUStore::archiveTask(ccu::Task task, ccu::TaskStatus task_status)
{
    mongocxx::client db_client{mongocxx::uri{}};

    //adding the task to the "task_archive" collection
    auto archive_collection = db_client[this->db_name]["task_archive"];
    bsoncxx::builder::stream::document document{};
    Json::Value task_json = task.toJson();
    //TODO: save the current timestamp
    task_json["task_status"] = task_status.status;
    for (auto robot_actions : task.robot_actions)
    {
        std::string robot_id = robot_actions.first;
        std::vector<std::string> completed_actions = task_status.completed_robot_actions[robot_id];

        Json::Value &robot_action_list = task_json["robot_actions"][robot_id];
        for (auto robot_action : robot_action_list)
        {
            std::string action_id = robot_action["id"].asString();
            if (std::find(completed_actions.begin(),
                          completed_actions.end(), action_id)
                          != completed_actions.end())
            {
                robot_action["status"] = "completed";
            }
        }
    }

    std::string task_string = Json::writeString(this->json_stream_builder, task_json);
    bsoncxx::document::value value = bsoncxx::from_json(task_string);
    archive_collection.insert_one(value.view());

    //removing the task from the "ongoing_tasks" collection
    auto ongoing_task_collection = db_client[this->db_name]["ongoing_tasks"];
    ongoing_task_collection.delete_one(bsoncxx::builder::stream::document{}
                                       << "task_id" << task.id
                                       << bsoncxx::builder::stream::finalize);

    //removing the task from the "ongoing_task_status" collection
    auto task_status_collection = db_client[this->db_name]["ongoing_task_status"];
    task_status_collection.delete_one(bsoncxx::builder::stream::document{}
                                      << "task_id" << task.id
                                      << bsoncxx::builder::stream::finalize);

    //removing the task from the "tasks" collection
    auto scheduled_task_collection = db_client[this->db_name]["tasks"];
    scheduled_task_collection.delete_one(bsoncxx::builder::stream::document{}
                                         << "id" << task.id
                                         << bsoncxx::builder::stream::finalize);
}

/**
 * Saves the given task id to a database as a new document under the "ongoing_tasks" collection
 *
 * @param task_id UUID representing the id of an already scheduled task
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
 * Adds a new task status document under the "ongoing_task_status" collection
 *
 * @param task_status task status description
 */
void CCUStore::addTaskStatus(const ccu::TaskStatus& task_status)
{
    mongocxx::client db_client{mongocxx::uri{}};
    auto collection = db_client[this->db_name]["ongoing_task_status"];
    bsoncxx::builder::stream::document document{};
    //TODO: save the current timestamp
    Json::Value task_status_json = task_status.toJson();
    std::string task_status_string = Json::writeString(this->json_stream_builder, task_status_json);
    bsoncxx::document::value value = bsoncxx::from_json(task_status_string);
    collection.insert_one(value.view());
}

/**
 * Saves an updated status for the given task under the "ongoing_task_status" collection
 *
 * @param task_status task status description
 */
void CCUStore::updateTaskStatus(const ccu::TaskStatus& task_status)
{

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
 * @param task_id UUID representing the id of a task
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
