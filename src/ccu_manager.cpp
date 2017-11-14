#include "ccu_manager.hpp"
#include <iostream>

CCUManager::CCUManager(ConfigParams config_params)
    : config_params_(config_params)
{
    for (size_t i=0; i<config_params.ropod_ids.size(); i++)
    {
        ropod_ids_.push_back(config_params.ropod_ids[i]);
    }

    ccu_node_ = new zyre::node_t("ccu");
    ccu_node_->start();
    ccu_node_->join("ROPOD");
    zclock_sleep(1000);
}

CCUManager::~CCUManager()
{
    ccu_node_->leave("ROPOD");
    ccu_node_->stop();
    zclock_sleep(1000);
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
