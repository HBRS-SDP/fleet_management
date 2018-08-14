#ifndef ELEVATOR_REQUESTS_HPP
#define ELEVATOR_REQUESTS_HPP

namespace ccu
{
    struct ElevatorRequests
    {
        int current_floor;
        int number_of_active_requests;
    };


    struct ElevatorRequest
    {
        int elevator_id;
        std::string operational_mode;
        int start_floor;
        int goal_floor;
        std::string query_id;
        std::string command;
        std::string task_id;
        std::string load;
        std::string robot_id;
        std::string status;
    struct Elevator
    {
        int elevator_id;
        int floor; //TODO: Need to match floors from toma messages to world model ones
        int calls;
        bool isAvailable;

        Json::Value toJson() const
        {
            Json::Value elevator_json;
            elevator_json["elevator_id"] = elevator_id;
            elevator_json["floor"] = floor;
            elevator_json["calls"] = calls;
            elevator_json["isAvailable"] = isAvailable;
            return elevator_json;
        }
    };
    };
}

#endif