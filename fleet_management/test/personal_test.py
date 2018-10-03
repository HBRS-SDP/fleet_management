from __future__ import print_function
import time
from fleet_management.structs.area import Area
from fleet_management.structs.area import Waypoint
from fleet_management.structs.robot import Robot
from fleet_management.db.ccu_store import CCUStore



if __name__ == '__main__':

    # create two Waypoints (one for each area)
    waypoint_A_dict = dict()
    waypoint_A_dict['semantic_id'] = '0'
    waypoint_A_dict['area_id'] = '1'
    waypoint_A_dict['x'] = '0'
    waypoint_A_dict['y'] = '0'

    waypoint_A = Waypoint()
    waypoint_A = Waypoint.from_dict(waypoint_A_dict)


    waypoint_B_dict = dict()
    waypoint_B_dict['semantic_id'] = '1'
    waypoint_B_dict['area_id'] = '2'
    waypoint_B_dict['x'] = '2'
    waypoint_B_dict['y'] = '2'

    waypoint_B = Waypoint()
    waypoint_B = Waypoint.from_dict(waypoint_B_dict)


    # create an area for each one of our Waypoints
    area_A_dict = dict()
    area_A_dict['id'] = 'area_A_id'
    area_A_dict['name'] = 'area_A'
    area_A_dict['floor_number'] = 1
    area_A_dict['type'] = ''
    area_A_waypoints = list()
    area_A_waypoints.append(waypoint_A)
    area_dict['waypoints'].append(area_A_waypoints)
    area_A = Area.from_dict(area_A_dict)

    area_B_dict = dict()
    area_B_dict['id'] = 'area_B_id'
    area_B_dict['name'] = 'area_B'
    area_B_dict['floor_number'] = 1
    area_B_dict['type'] = ''
    area_B_waypoints = list()
    area_B_waypoints.append(waypoint_B)
    area_dict['waypoints'].append(area_B_waypoints)
    area_B = Area.from_dict(area_B_dict)


    robot_A_dict = dict()

    status_dict = dict()
    status_dict['robot_id'] = 'A'
    status_dict['current_location'] = area_A
    status_dict['current_operation'] = 'hangout'
    status_dict['status'] = 'idle'
    status_dict['available'] = 'na'
    status_dict['battery_status'] = 'voll'

    robot_A_dict['robot_id'] = 'A'
    robot_A_dict['schedule'] = 'N/A'
    robot_A_dict['status'] = status_dict

    robotA = Robot.from_dict(robot_A_dict)

    print('Getting our CCUStore')
    ccu_store = CCUStore('ropod_personal_data_test')
    ccu_store.add_robot(robot_A)

    print("trying to get bots from the ccu")
    robots = get_robots()
    print("here are those bots I tried to get:\n", robots);
