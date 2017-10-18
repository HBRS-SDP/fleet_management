#ifndef CCU_MANAGER_HPP
#define CCU_MANAGER_HPP

#include <string>
#include <vector>
#include <memory>
#include <json/json.h>

#include "extern/zyre/node.hpp"
#include "ropod_common/msg_constants.hpp"
#include "config/config_params.hpp"

class CCUManager
{
public:
    CCUManager(ConfigParams config_params);
    ~CCUManager();
    bool sendNavigationCommand(std::string ropod_id, std::string waypoint_id);
    bool sendDockingCommand(std::string ropod_id, std::string object_id);
    bool sendUndockingCommand(std::string ropod_id);
    bool sendStopCommand(std::string ropod_id, int milliseconds);
    bool sendElevatorOpenDoorCommand();
    bool sendElevatorCloseDoorCommand();
    bool sendElevatorGoToFloorCommand(int floor_number);
private:
    zmsg_t* string_to_zmsg(std::string msg);

    ConfigParams config_params_;

    std::vector<std::string> ropod_ids_;
    std::string elevator_id_;

    zyre::node_t *ccu_node_;

    Json::StreamWriterBuilder json_stream_builder_;
};

#endif
