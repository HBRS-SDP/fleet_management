from fleet_management.structs.action import Action
from fleet_management.structs.area import Area


class TaskPlanner(object):
    '''An interface for generating ROPOD task plans

    @author Alex Mitrevski
    @maintainer Alex Mitrevski, Argentina Ortega Sainz
    @contact aleksandar.mitrevski@h-brs.de, argentina.ortega@h-brs.de
    '''

    '''Generates a task plan based on the given task request and returns a list of
    fleet_management.structs.action.Action objects representing the plan's actions

    @param task_request a fleet_management.structs.task.TaskRequest object

    '''
    @staticmethod
    def get_task_plan(task_request, path_planner=None):
        actions = list()
        if task_request.cart_type == 'mobidik':

            go_to_pickup_pose = Action()
            go_to_pickup_pose.type = 'GOTO'
            go_to_pickup_pose.areas.append(task_request.pickup_pose)

            dock_cart = Action()
            dock_cart.type = 'DOCK'
            dock_cart.areas.append(task_request.pickup_pose)

            go_to_delivery_pose = Action()
            go_to_delivery_pose.type = 'GOTO'
            go_to_delivery_pose.areas.append(task_request.delivery_pose)

            undock = Action()
            undock.type = 'UNDOCK'
            undock.areas.append(task_request.delivery_pose)

            go_to_charging_station = Action()
            go_to_charging_station.type = 'GOTO'

            charging_station = Area()
            charging_station = path_planner.get_area('AMK_A_L-1_RoomBU21_LA1')
            charging_station.floor_number = -1
            go_to_charging_station.areas.append(charging_station)

            actions.append(go_to_pickup_pose)
            actions.append(dock_cart)
            actions.append(go_to_delivery_pose)
            actions.append(undock)
            actions.append(go_to_charging_station)
        elif task_request.cart_type == 'sickbed':
            # TBD
            pass

        expanded_task_plan = TaskPlanner.__expand_task_plan(actions, path_planner)
        return expanded_task_plan

    '''Adds additional actions to the task plan (e.g. elevator calls)
    so that the plan is actually executable

    @param task_plan a list of fleet_management.structs.action.Action objects

    '''
    @staticmethod
    def __expand_task_plan(task_plan, path_planner=None):
        expanded_task_plan = list()

        # we assume that the first action of a task is always a GOTO action;
        # however, we only expand the plan starting from the second action
        # because the robots for a task haven't been chosen yet,
        # so we don't know what the start location is
        expanded_task_plan.append(task_plan[0])
        previous_location = task_plan[0].areas[-1]
        for i in range(1, len(task_plan)):
            action = task_plan[i]
            if action.type != 'GOTO':
                expanded_task_plan.append(action)
            else:
                destination = action.areas[0]
                path_plan = path_planner.get_path_plan(previous_location, destination)

                # if both locations are on the same floor, we can simply take the
                # path plan as the areas that have to be visited in a single GOTO action;
                # the situation is more complicated when the start and end location
                # are on two different floors, as we then have to insert elevator
                # request and entering/exiting actions in the task plan
                if previous_location.floor_number == destination.floor_number:
                    action.areas = path_plan
                    expanded_task_plan.append(action)
                else:
                    # when the start and destination locations are on different floors,
                    # the path plan P = <A1, A2, ..., An> has two subsequences
                    # P1 = <A1, A2, ..., Ak> and P2 = <Ak+1, Ak+2, ..., An>,
                    # where the areas in P1 are on the same floor as the start location
                    # and the areas in P2 are on the same floor as the destination location
                    start_floor = previous_location.floor_number
                    end_floor = destination.floor_number

                    start_floor_areas = list()
                    goal_floor_areas = list()
                    for area in path_plan:
                        if area.floor_number == start_floor:
                            start_floor_areas.append(area)
                        else:
                            goal_floor_areas.append(area)

                    # this is the action for going through the areas in P1
                    start_floor_go_to = Action()
                    start_floor_go_to.type = 'GOTO'
                    start_floor_go_to.areas = start_floor_areas
                    expanded_task_plan.append(start_floor_go_to)

                    # action for requesting an elevator
                    request_elevator = Action()
                    request_elevator.type = 'REQUEST_ELEVATOR'
                    request_elevator.start_floor = start_floor
                    request_elevator.goal_floor = end_floor
                    expanded_task_plan.append(request_elevator)

                    # action for entering the elevator
                    enter_elevator = Action()
                    enter_elevator.type = 'ENTER_ELEVATOR'
                    enter_elevator.level = start_floor
                    expanded_task_plan.append(enter_elevator)

                    # action for exiting the elevator
                    exit_elevator = Action()
                    exit_elevator.type = 'EXIT_ELEVATOR'
                    exit_elevator.level = end_floor
                    expanded_task_plan.append(exit_elevator)

                    # this is the action for going through the areas in P2
                    end_floor_go_to = Action()
                    end_floor_go_to.type = 'GOTO'
                    end_floor_go_to.areas = goal_floor_areas
                    expanded_task_plan.append(end_floor_go_to)

                previous_location = destination

        return expanded_task_plan
