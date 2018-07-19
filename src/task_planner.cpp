#include "task_planner.hpp"

namespace ccu
{
    std::vector<Action> TaskPlanner::getTaskPlan(const TaskRequest& task_request) const
    {
        std::vector<Action> action_list;

        //for now, we only have have a fixed number of high-level task plans
        if (task_request.cart_type == "mobidik")
        {
            Action go_to_pickup_pose;
            go_to_pickup_pose.type = "go_to";
            go_to_pickup_pose.areas.push_back(task_request.pickup_pose);

            Action dock_cart;
            dock_cart.type = "dock";
            dock_cart.areas.push_back(task_request.pickup_pose);

            Action go_to_delivery_pose;
            go_to_delivery_pose.type = "go_to";
            go_to_delivery_pose.areas.push_back(task_request.delivery_pose);

            Action undock;
            undock.type = "undock";
            undock.areas.push_back(task_request.delivery_pose);

            Action go_to_charging_station;
            go_to_charging_station.type = "go_to_charging_station";

            Waypoint charging_station;
            charging_station.semantic_id = "charging_station";
            go_to_charging_station.waypoints.push_back(charging_station);

            action_list.push_back(go_to_pickup_pose);
            action_list.push_back(dock_cart);
            action_list.push_back(go_to_delivery_pose);
            action_list.push_back(undock);
            action_list.push_back(go_to_charging_station);
        }
        else if (task_request.cart_type == "sickbed")
        {
            //TBD
        }

        return action_list;
    }
}
