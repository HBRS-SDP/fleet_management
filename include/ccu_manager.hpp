#ifndef CCU_MANAGER_HPP
#define CCU_MANAGER_HPP

#include <string>
#include <vector>
#include <zmq.hpp>
#include <json/json.h>

#include "config/config_params.hpp"

class CCUManager
{
public:
    CCUManager(ConfigParams config_params);
private:
    ConfigParams config_params_;

    std::map<std::string, zmq::context_t*> zmq_contexts_;
    std::map<std::string, zmq::socket_t*> sockets_;
};

#endif
