#ifndef CCU_MANAGER_HPP
#define CCU_MANAGER_HPP

#include <string>
#include <vector>
#include <memory>
#include <zmq.hpp>
#include <json/json.h>

#include "config/config_params.hpp"
#include "msg_params.hpp"

class CCUManager
{
public:
    CCUManager(ConfigParams config_params);
    bool sendNavigationCommand(std::string ropod_ip, std::string waypoint_id);
    bool sendDockingCommand(std::string ropod_ip, std::string object_id);
    bool sendUndockingCommand(std::string ropod_ip);
    bool sendStopCommand(std::string ropod_ip, int milliseconds);
private:
    ConfigParams config_params_;

    std::map<std::string, std::shared_ptr<zmq::context_t>> zmq_contexts_;
    std::map<std::string, std::shared_ptr<zmq::socket_t>> sockets_;

    Json::StreamWriterBuilder json_stream_builder_;
};

#endif
