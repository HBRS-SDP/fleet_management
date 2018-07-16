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
        int robot_id;
        std::string status;
    };
}

#endif