#include "ccu_manager.hpp"
#include <iostream>

CCUManager::CCUManager(ConfigParams config_params)
    : config_params_(config_params)
{
    for (size_t i=0; i<config_params.ropod_ids.size(); i++)
    {
        ropod_ids_.push_back(config_params.ropod_ids[i]);
    }
    elevator_id_ = config_params.elevator_id;

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

bool CCUManager::sendNavigationCommand(std::string ropod_id, std::string waypoint_id)
{
    Json::Value root;
    root["conversation_id"] = MsgConstants::GO_TO_GOAL;
    root["performative"] = MsgConstants::REQUEST;
    root["content"]["waypoint_id"] = waypoint_id;
    std::string msg = Json::writeString(json_stream_builder_, root);

    std::cout << "sending navigation request" << std::endl;
    zmsg_t* message = string_to_zmsg(msg);
    ccu_node_->shout("ROPOD", message);

    return true;
}

bool CCUManager::sendDockingCommand(std::string ropod_id, std::string object_id)
{
    Json::Value root;
    root["conversation_id"] = MsgConstants::DOCK;
    root["performative"] = MsgConstants::REQUEST;
    root["content"]["object_id"] = object_id;
    std::string msg = Json::writeString(json_stream_builder_, root);

    std::cout << "sending docking request" << std::endl;
    zmsg_t* message = string_to_zmsg(msg);
    ccu_node_->shout("ROPOD", message);

    return true;
}

bool CCUManager::sendUndockingCommand(std::string ropod_id)
{
    Json::Value root;
    root["conversation_id"] = MsgConstants::UNDOCK;
    root["performative"] = MsgConstants::REQUEST;
    std::string msg = Json::writeString(json_stream_builder_, root);

    std::cout << "sending undocking request" << std::endl;
    zmsg_t* message = string_to_zmsg(msg);
    ccu_node_->shout("ROPOD", message);

    return true;
}

bool CCUManager::sendStopCommand(std::string ropod_id, int milliseconds)
{
    Json::Value root;
    root["conversation_id"] = MsgConstants::STOP;
    root["performative"] = MsgConstants::REQUEST;
    root["content"]["duration"] = milliseconds;
    std::string msg = Json::writeString(json_stream_builder_, root);

    std::cout << "sending stop request" << std::endl;
    zmsg_t* message = string_to_zmsg(msg);
    ccu_node_->shout("ROPOD", message);

    return true;
}

bool CCUManager::sendElevatorOpenDoorCommand()
{
    Json::Value root;
    root["conversation_id"] = MsgConstants::ELEVATOR_OPEN_DOOR;
    root["performative"] = MsgConstants::REQUEST;
    std::string msg = Json::writeString(json_stream_builder_, root);

    std::cout << "sending elevator open door request" << std::endl;
    zmsg_t* message = string_to_zmsg(msg);
    ccu_node_->shout("ROPOD", message);

    return true;
}

bool CCUManager::sendElevatorCloseDoorCommand()
{
    Json::Value root;
    root["conversation_id"] = MsgConstants::ELEVATOR_CLOSE_DOOR;
    root["performative"] = MsgConstants::REQUEST;
    std::string msg = Json::writeString(json_stream_builder_, root);

    std::cout << "sending elevator close door request" << std::endl;
    zmsg_t* message = string_to_zmsg(msg);
    ccu_node_->shout("ROPOD", message);

    return true;
}

bool CCUManager::sendElevatorGoToFloorCommand(int floor_number)
{
    Json::Value root;
    root["conversation_id"] = MsgConstants::ELEVATOR_GO_TO_FLOOR;
    root["performative"] = MsgConstants::REQUEST;
    root["content"]["floor_number"] = floor_number;
    std::string msg = Json::writeString(json_stream_builder_, root);

    std::cout << "sending elevator go to floor request" << std::endl;
    zmsg_t* message = string_to_zmsg(msg);
    ccu_node_->shout("ROPOD", message);

    return true;
}
