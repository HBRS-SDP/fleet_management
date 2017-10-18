#ifndef CCU_MANAGER_HPP
#define CCU_MANAGER_HPP

#include <string>
#include <vector>
#include <memory>
#include <json/json.h>

#include "extern/zyre/node.hpp"
#include "config/config_params.hpp"

class CCUManager
{
public:
    CCUManager(ConfigParams config_params);
    ~CCUManager();
    bool sendGOTOCommand(const std::string &waypoint_id);
    bool sendElevatorCommand(const std::string &elevator_command);
    bool sendCoordinationCommand(const std::string &coordination_command);

private:
    Json::Value getHeader(const std::string &command);
    void shout(const Json::Value &root);

private:
    zmsg_t* string_to_zmsg(std::string msg);

    ConfigParams config_params_;

    std::vector<std::string> ropod_ids_;
    std::string elevator_id_;

    zyre::node_t *ccu_node_;

    Json::StreamWriterBuilder json_stream_builder_;
};

#endif
