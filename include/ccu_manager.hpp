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
    std::vector<zmq::context_t*> zmq_contexts_;
    std::vector<zmq::socket_t*> sockets_;
};

#endif
