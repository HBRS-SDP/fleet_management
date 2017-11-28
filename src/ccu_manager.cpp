#include "ccu_manager.hpp"
#include <iostream>
#include <sstream>

static void receiveLoop(zsock_t *pipe, void *args);

CCUManager::CCUManager(ConfigParams config_params)
    : config_params_(config_params)
{
    for (size_t i=0; i<config_params.ropod_ids.size(); i++)
    {
        ropod_ids_.push_back(config_params.ropod_ids[i]);
    }

    ccu_node_ = new zyre::node_t("ccu");
    ccu_node_->start();
    ccu_node_->join(config_params_.zyre_group_name);

    actor_ = zactor_new(receiveLoop, this);
    zclock_sleep(1000);
}

CCUManager::~CCUManager()
{
    ccu_node_->leave(config_params_.zyre_group_name);
    ccu_node_->stop();
    zclock_sleep(1000);
    zactor_destroy(&actor_);
    delete ccu_node_;
}

zmsg_t* CCUManager::string_to_zmsg(std::string msg)
{
    zmsg_t* message = zmsg_new();
    zframe_t *frame = zframe_new(msg.c_str(), msg.size());
    zmsg_prepend(message, &frame);
    return message;
}

Json::Value CCUManager::getHeader(const std::string &command)
{
    Json::Value root;
    root["type"] = command;
    root["version"] = config_params_.message_version;
    root["metamodel"] = "ropod-msg-schema.json";
    zuuid_t * uuid = zuuid_new();
    const char * uuid_str = zuuid_str_canonical(uuid);
    root["msg_id"] = uuid_str;
    zuuid_destroy(&uuid);
    char * timestr = zclock_timestr();
    root["timestamp"] = timestr;
    zstr_free(&timestr);
    return root;
}

void CCUManager::shout(const Json::Value &root)
{
    std::string msg = Json::writeString(json_stream_builder_, root);
    zmsg_t* message = string_to_zmsg(msg);
    ccu_node_->shout(config_params_.zyre_group_name, message);
}

bool CCUManager::sendGOTOCommand(const std::string &waypoint_id)
{
    Json::Value root;
    root["header"] = getHeader("CMD");

    root["payload"]["metamodel"] = "ropod-demo-cmd-schema.json";
    Json::Value &commandList = root["payload"]["commandList"];
    Json::Value command;
    command["command"] = "GOTO";
    command["location"] = waypoint_id;
    commandList.append(command);

    shout(root);
    return true;
}

bool CCUManager::sendElevatorCommand(const std::string &elevator_command)
{
    Json::Value root;
    root["header"] = getHeader("CMD");

    root["payload"]["metamodel"] = "ropod-demo-cmd-schema.json";
    Json::Value &commandList = root["payload"]["commandList"];
    Json::Value command;
    command["command"] = elevator_command;
    commandList.append(command);

    shout(root);
    return true;
}

bool CCUManager::sendCoordinationCommand(const std::string &coordination_command)
{
    Json::Value root;
    root["header"] = getHeader("CMD");

    root["payload"]["metamodel"] = "ropod-demo-cmd-schema.json";
    Json::Value &commandList = root["payload"]["commandList"];
    Json::Value command;
    command["command"] = coordination_command;
    commandList.append(command);

    shout(root);
    return true;
}

void CCUManager::parseInputMessage(zmsg_t *msg)
{
    char *event = zmsg_popstr (msg);
    char *peer = zmsg_popstr (msg);
    char *name = zmsg_popstr (msg);
    char *group = zmsg_popstr (msg);
    char *message = zmsg_popstr (msg);

    Json::Value root;
    std::stringstream msg_stream;
    msg_stream << message;
    if (streq(event, "SHOUT"))
    {
        Json::CharReaderBuilder reader_builder;
        std::string errors;
        bool ok = Json::parseFromStream(reader_builder, msg_stream, &root, &errors);
        std::string type = root["header"]["type"].asString();
        if (type == "RobotPose2D")
        {
            parseRobotPoseMessage(root);
        }
        else if (type == "progress")
        {
            parseProgressMessage(root);
        }
    }

    free (event);
    free (peer);
    free (name);
    free (group);
    free (message);
    zmsg_destroy (&msg);
}

void CCUManager::parseRobotPoseMessage(const Json::Value &root)
{
    const Json::Value payload = root["payload"];
    const std::string robot = payload["robotId"].asString();
    const std::string referenceId = payload["pose"]["referenceId"].asString();
    const double x = payload["pose"]["x"].asDouble();
    const double y = payload["pose"]["y"].asDouble();
    const double theta = payload["pose"]["theta"].asDouble();
    std::cout << "\r" << robot << ": x: " << x << " y: " << y << " theta: " << theta << std::flush;
}

void CCUManager::parseProgressMessage(const Json::Value &root)
{

}

static void receiveLoop(zsock_t *pipe, void *args)
{
    CCUManager * ccu_manager = (CCUManager*)(args);
    zyre::node_t * node = ccu_manager->getNode();

    zsock_signal(pipe, 0);
    bool terminated = false;
    // this poller will listen to messages that the node receives
    // AND messages received by this actor on pipe
    zpoller_t *poller = zpoller_new (pipe, node->socket(), NULL);
    while (!terminated)
    {
        void *which = zpoller_wait (poller, -1); // no timeout
        if (which == pipe) // message sent to the actor
        {
            zmsg_t *msg = zmsg_recv (which);
            if (!msg)
                break;              //  Interrupted
            char *command = zmsg_popstr (msg);
            if (streq (command, "$TERM")) {
                terminated = true;
            }
            else {
                std::cerr << "invalid message to actor" << std::endl;
                assert (false);
            }
            free (command);
            zmsg_destroy (&msg);
        }
        else if (which == node->socket()) // message sent to the node
        {
            zmsg_t *msg = zmsg_recv (which);
            ccu_manager->parseInputMessage(msg);
        }
    }
    zpoller_destroy (&poller);
}
