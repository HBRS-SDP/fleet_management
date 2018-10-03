from __future__ import print_function
import time
from fleet_management.structs.area import Area
from fleet_management.structs.area import Waypoint
from fleet_management.structs.robot import Robot
from fleet_management.structs.status import RobotStatus
from fleet_management.db.ccu_store import CCUStore



if __name__ == '__main__':

    # create two Waypoints (one for each area)
    waypoint_A = Waypoint()
    waypoint_A.semantic_id = '0'
    waypoint_A.area_id = '1'
    waypoint_A.x = '1'
    waypoint_A.y = '1'

    waypoint_B = Waypoint()
    waypoint_B.semantic_id = '1'
    waypoint_B.area_id = '2'
    waypoint_B.x = '2'
    waypoint_B.y = '2'

    # create an area for each one of our Waypoints
    area_A = Area()
    area_A.id = list('area_A_id')
    area_A.name = 'area_A'
    area_A.floor_number = 1
    area_A.type = ''
    area_A.waypoints = list()
    area_A.waypoints.append(waypoint_A)

    area_B = Area()
    area_B.id = list('area_B_id')
    area_B.name = 'area_B'
    area_B.floor_number = 1
    area_B.type = ''
    area_B.waypoints = list()
    area_B.waypoints.append(waypoint_B)

    status = RobotStatus()
    status.robot_id = 'A'
    status.current_location = area_A
    status.current_operation = 'hangout'
    status.status = 'idle'
    status.available = 'na'
    status.battery_status = 'voll'

    robot_A = Robot()

    robot_A.robot_id = 'A'
    robot_A.schedule = 'N/A'
    robot_A.status = status

    print('Getting our CCUStore')
    ccu_store = CCUStore('ropod_personal_data_test')
    ccu_store.add_robot(robot_A)

    print("trying to get bots from the ccu")
    robots = ccu_store.get_robots()
    print("here are those bots I tried to get:\n", robots);
