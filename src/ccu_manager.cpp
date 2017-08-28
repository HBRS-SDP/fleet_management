#include "ccu_manager.hpp"
#include <iostream>

CCUManager::CCUManager(ConfigParams config_params)
    : config_params_(config_params)
{
    for (size_t i=0; i<config_params.ropod_ips.size(); i++)
    {
        std::shared_ptr<zmq::context_t> context = std::make_shared<zmq::context_t>(1);
        std::shared_ptr<zmq::socket_t> socket = std::make_shared<zmq::socket_t>(*context, ZMQ_REQ);
        // socket->bind("tcp://*:5555");
        socket->connect(std::string("tcp://" + config_params.ropod_ips[i] + ":" + config_params.ropod_ports[i]).c_str());
        // socket->bind("tcp://*:5555");

        zmq_contexts_.insert(std::make_pair(config_params.ropod_ips[i], context));
        sockets_.insert(std::make_pair(config_params.ropod_ips[i], socket));
    }

    elevator_context_ = std::make_shared<zmq::context_t>(1);
    elevator_socket_ = std::make_shared<zmq::socket_t>(*elevator_context_, ZMQ_REQ);
    elevator_socket_->connect(std::string("tcp://" + config_params.elevator_ip + ":" + config_params.elevator_port).c_str());
}

bool CCUManager::sendNavigationCommand(std::string ropod_ip, std::string waypoint_id)
{
    Json::Value root;
    root["conversation_id"] = MsgConversationIds::GO_TO_GOAL;
    root["performative"] = MsgPerformativeIds::REQUEST;
    root["content"]["waypoint_id"] = waypoint_id;
    std::string msg = Json::writeString(json_stream_builder_, root);

    std::cout << "sending navigation request" << std::endl;
    zmq::message_t request(msg.size());
    memcpy(request.data(), msg.c_str(), msg.size());
    sockets_[ropod_ip]->send(request);

    std::cout << "waiting for reply..." << std::endl;
    zmq::message_t reply;
    sockets_[ropod_ip]->recv(&reply);
    std::cout << "reply received..." << std::endl;

    //TODO: adjust the return value based on the reply

    return true;
}

bool CCUManager::sendDockingCommand(std::string ropod_ip, std::string object_id)
{
    Json::Value root;
    root["conversation_id"] = MsgConversationIds::DOCK;
    root["performative"] = MsgPerformativeIds::REQUEST;
    root["content"]["object_id"] = object_id;
    std::string msg = Json::writeString(json_stream_builder_, root);

    zmq::message_t request(msg.size());
    memcpy(request.data(), msg.c_str(), msg.size());
    sockets_[ropod_ip]->send(request);

    zmq::message_t reply;
    sockets_[ropod_ip]->recv(&reply);

    //TODO: adjust the return value based on the reply

    return true;
}

bool CCUManager::sendUndockingCommand(std::string ropod_ip)
{
    Json::Value root;
    root["conversation_id"] = MsgConversationIds::UNDOCK;
    root["performative"] = MsgPerformativeIds::REQUEST;
    std::string msg = Json::writeString(json_stream_builder_, root);

    zmq::message_t request(msg.size());
    memcpy(request.data(), msg.c_str(), msg.size());
    sockets_[ropod_ip]->send(request);

    zmq::message_t reply;
    sockets_[ropod_ip]->recv(&reply);

    //TODO: adjust the return value based on the reply

    return true;
}

bool CCUManager::sendStopCommand(std::string ropod_ip, int milliseconds)
{
    Json::Value root;
    root["conversation_id"] = MsgConversationIds::STOP;
    root["performative"] = MsgPerformativeIds::REQUEST;
    root["content"]["duration"] = milliseconds;
    std::string msg = Json::writeString(json_stream_builder_, root);

    zmq::message_t request(msg.size());
    memcpy(request.data(), msg.c_str(), msg.size());
    sockets_[ropod_ip]->send(request);

    zmq::message_t reply;
    sockets_[ropod_ip]->recv(&reply);

    //TODO: adjust the return value based on the reply

    return true;
}

bool CCUManager::sendElevatorOpenDoorCommand()
{
    Json::Value root;
    root["conversation_id"] = MsgConversationIds::ELEVATOR_OPEN_DOOR;
    root["performative"] = MsgPerformativeIds::REQUEST;
    std::string msg = Json::writeString(json_stream_builder_, root);

    zmq::message_t request(msg.size());
    memcpy(request.data(), msg.c_str(), msg.size());
    elevator_socket_->send(request);

    zmq::message_t reply;
    elevator_socket_->recv(&reply);

    //TODO: adjust the return value based on the reply

    return true;
}

bool CCUManager::sendElevatorCloseDoorCommand()
{
    Json::Value root;
    root["conversation_id"] = MsgConversationIds::ELEVATOR_CLOSE_DOOR;
    root["performative"] = MsgPerformativeIds::REQUEST;
    std::string msg = Json::writeString(json_stream_builder_, root);

    zmq::message_t request(msg.size());
    memcpy(request.data(), msg.c_str(), msg.size());
    elevator_socket_->send(request);

    zmq::message_t reply;
    elevator_socket_->recv(&reply);

    //TODO: adjust the return value based on the reply

    return true;
}

bool CCUManager::sendElevatorGoToFloorCommand(int floor_number)
{
    Json::Value root;
    root["conversation_id"] = MsgConversationIds::ELEVATOR_GO_TO_FLOOR;
    root["performative"] = MsgPerformativeIds::REQUEST;
    root["content"]["floor_number"] = floor_number;
    std::string msg = Json::writeString(json_stream_builder_, root);

    zmq::message_t request(msg.size());
    memcpy(request.data(), msg.c_str(), msg.size());
    elevator_socket_->send(request);

    zmq::message_t reply;
    elevator_socket_->recv(&reply);

    //TODO: adjust the return value based on the reply

    return true;
}
