#include "task_planner.hpp"

namespace ccu
{
    std::vector<Action> TaskPlanner::getTaskPlan(const TaskRequest& task_request)
    {
        std::vector<Action> action_list;

        //for now, we only have have a fixed number of high-level task plans
        if (task_request.cart_type == "mobidik")
        {
            Action go_to_pickup_pose;
            go_to_pickup_pose.type = "GOTO";
            go_to_pickup_pose.areas.push_back(task_request.pickup_pose);

            Action dock_cart;
            dock_cart.type = "DOCK";
            dock_cart.areas.push_back(task_request.pickup_pose);

            Action go_to_delivery_pose;
            go_to_delivery_pose.type = "GOTO";
            go_to_delivery_pose.areas.push_back(task_request.delivery_pose);

            Action undock;
            undock.type = "UNDOCK";
            undock.areas.push_back(task_request.delivery_pose);

            Action go_to_charging_station;
            go_to_charging_station.type = "GOTO";

            Area charging_station;
            charging_station.name = "charging_station";
            charging_station.floor_number = 0;
            go_to_charging_station.areas.push_back(charging_station);

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

        std::vector<Action> expanded_task_plan = TaskPlanner::expandTaskPlan(action_list);
        return expanded_task_plan;
    }

    std::vector<Action> TaskPlanner::expandTaskPlan(const std::vector<Action>& task_plan)
    {
        std::vector<Action> expanded_task_plan;

        //we assume that the first action of a task is always a go to action;
        //however, we only expand the plan starting from the second action
        //because the robots for a task haven't been chosen yet,
        //so we don't know what the start location is
        expanded_task_plan.push_back(task_plan[0]);
        Area previous_location = task_plan[0].areas[task_plan[0].areas.size()-1];
        for (int i=1; i<task_plan.size(); i++)
        {
            Action action = task_plan[i];
            if (action.type != "GOTO")
            {
                expanded_task_plan.push_back(action);
            }
            else
            {
                Area destination = action.areas[0];
                std::vector<Area> areas = PathPlanner::getPathPlan(previous_location, destination);

                //if both locations are on the same floor, we can simply take the
                //path plan as the areas that have to be visited in a single GOTO action;
                //the situation is more complicated when the start and end location
                //are on two different floors, as we then have to insert elevator
                //request and entering/exiting actions in the task plan
                if (previous_location.floor_number == destination.floor_number)
                {
                    action.areas = areas;
                    expanded_task_plan.push_back(action);
                }
                else
                {
                    //when the start and destination locations are on different floors,
                    //the path plan P = <A1, A2, ..., An> has two subsequences
                    //P1 = <A1, A2, ..., Ak> and P2 = <Ak+1, Ak+2, ..., An>,
                    //where the areas in P1 are on the same floor as start location
                    //and the areas in P2 are on the same floor as the destination location
                    int start_floor = previous_location.floor_number;
                    int end_floor = destination.floor_number;

                    std::vector<Area> start_floor_areas;
                    std::vector<Area> end_floor_areas;

                    int area_idx = 0;
                    for (Area area : areas)
                    {
                        if (area.floor_number == start_floor)
                        {
                            start_floor_areas.push_back(area);
                        }
                        else
                        {
                            end_floor_areas.push_back(area);
                        }
                    }

                    //this is the action for going through the areas in P1
                    Action start_floor_go_to;
                    start_floor_go_to.type = "GOTO";
                    start_floor_go_to.areas = start_floor_areas;
                    expanded_task_plan.push_back(start_floor_go_to);

                    //action for requesting an elevator
                    Action request_elevator;
                    request_elevator.type = "REQUEST_ELEVATOR";
                    request_elevator.start_floor = start_floor;
                    request_elevator.goal_floor = end_floor;
                    expanded_task_plan.push_back(request_elevator);

                    //action for entering the elevator
                    Action enter_elevator;
                    enter_elevator.type = "ENTER_ELEVATOR";
                    enter_elevator.level = start_floor;
                    expanded_task_plan.push_back(enter_elevator);

                    //action for exiting the elevator
                    Action exit_elevator;
                    exit_elevator.type = "EXIT_ELEVATOR";
                    exit_elevator.level = end_floor;
                    expanded_task_plan.push_back(exit_elevator);

                    //this is the action for going through the areas in P2
                    Action end_floor_go_to;
                    end_floor_go_to.type = "GOTO";
                    end_floor_go_to.areas = end_floor_areas;
                    expanded_task_plan.push_back(end_floor_go_to);
                }
                previous_location = destination;
            }
        }
        return expanded_task_plan;
    }
}
