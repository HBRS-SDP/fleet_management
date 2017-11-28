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
    /*
     * shouts a 'GOTO' command
     * waypoint_id: name of semantic location on the map
     */
    bool sendGOTOCommand(const std::string &waypoint_id);

    /*
     * shouts an elevator command
     * elevator_command:
     *     ENTER_ELEVATOR
     *     EXIT_ELEVATOR
     */
    bool sendElevatorCommand(const std::string &elevator_command);

    /*
     * shouts a coordination command
     * coordination_command:
     *      PAUSE
     *      RESUME
     */
    bool sendCoordinationCommand(const std::string &coordination_command);

    /*
     * Parses zeromq messages passed as JSON strings
     *
     * msg: message shouted by other nodes as JSON string
     *
     * this function has ownership over msg, so it
     * deletes it after parsing
     */
    void parseInputMessage(zmsg_t *msg);

    /*
     * Returns zyre node
     */
    zyre::node_t * getNode() {return ccu_node_;}

private:
    Json::Value getHeader(const std::string &command);
    void shout(const Json::Value &root);

    zmsg_t* string_to_zmsg(std::string msg);

    void parseRobotPoseMessage(const Json::Value &root);
    void parseProgressMessage(const Json::Value &root);

private:

    ConfigParams config_params_;

    std::vector<std::string> ropod_ids_;
    std::string elevator_id_;

    zyre::node_t *ccu_node_;
    zactor_t *actor_;

    Json::StreamWriterBuilder json_stream_builder_;
};

#endif
