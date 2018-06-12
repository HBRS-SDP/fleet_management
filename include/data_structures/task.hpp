#ifndef TASK_HPP
#define TASK_HPP

#include <string>
#include <vector>
#include <map>

namespace task
{
    struct Waypoint
    {
        std::string semantic_id;
        std::string area_id;
        int floor_number;
        int x;
        int y;
    };

    struct Action
    {
        std::string id;
        std::vector<Waypoint> waypoints;
        std::string execution_status; // pending, in progress, etc.
        float eta;
    };

    struct TaskRequest
    {
        Waypoint pickup_pose;
        Waypoint delivery_pose;
        float start_time;
        std::string user_id;
        std::string cart_type;
    };

    struct Task
    {
        int id;
        std::map<std::string, std::vector<Action>> robot_actions; //TODO: consider a differnt data structure for the list of actions
                                                                  //so that it's easier to expand it if necessary
        std::vector<std::string> team_robot_ids;
        float start_time;
    };

    struct MissionStatus
    {
        int mission_id;
        std::map<int, Waypoint> robot_waypoints;
        std::map<int, Action> current_robot_actions;
        std::map<int, std::vector<Action>> completed_robot_actions;
        std::map<int, int> robot_floor;
        std::map<int, float> task_duration;
        std::map<int, float> mission_duration;
    };

    struct RobotTask
    {
        float start_time;
        float estimated_end_time;
    };

    struct RobotStatus
    {
        bool operational;
    };

    struct ElevatorRequests
    {
        int current_floor;
        int number_of_active_requests;
    };
}

#endif
