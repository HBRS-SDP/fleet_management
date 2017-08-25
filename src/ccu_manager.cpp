#include "ccu_manager.hpp"

CCUManager::CCUManager(ConfigParams config_params)
    : config_params_(config_params)
{
    for (size_t i=0; i<config_params.ropod_ips.size(); i++)
    {
        std::shared_ptr<zmq::context_t> context = std::make_shared<zmq::context_t>(1);
        std::shared_ptr<zmq::socket_t> socket = std::make_shared<zmq::socket_t>(*context, ZMQ_REQ);
        socket->bind(std::string("tcp://" + config_params.ropod_ips[i] + ":" + config_params.ropod_ports[i]).c_str());

        zmq_contexts_.insert(std::make_pair(config_params.ropod_ips[i], context));
        sockets_.insert(std::make_pair(config_params.ropod_ips[i], socket));
    }
}

bool CCUManager::sendNavigationCommand(std::string ropod_ip, std::string waypoint_id)
{
    Json::Value root;
    root["conversation_id"] = MsgConversationIds::GO_TO_GOAL;
    root["performative"] = MsgPerformativeIds::REQUEST;
    root["content"]["waypoint_id"] = waypoint_id;
    std::string msg = Json::writeString(json_stream_builder_, root);

    zmq::message_t request(msg.size());
    memcpy(request.data(), msg.c_str(), msg.size());
    sockets_[ropod_ip]->send(request);

    zmq::message_t reply;
    sockets_[ropod_ip]->recv(&reply);

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
