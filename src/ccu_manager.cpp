#include "ccu_manager.hpp"

CCUManager::CCUManager(ConfigParams config_params)
{
    for (size_t i=0; i<config_params.ropod_ips.size(); i++)
    {
        zmq::context_t *context = new zmq::context_t(1);
        zmq::socket_t *socket = new zmq::socket_t(*context, ZMQ_PUB);
        socket->bind(std::string("tcp://" + config_params.ropod_ips[i] + ":" + config_params.ropod_ports[i]).c_str());

        zmq_contexts_.push_back(context);
        sockets_.push_back(socket);
    }
}
