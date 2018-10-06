from __future__ import print_function
import time
import json

from fleet_management.structs.robot import Robot
from fleet_management.structs.area import Area
from fleet_management.structs.area import Waypoint
from fleet_management.structs.status import RobotStatus
from fleet_management.db.ccu_store import CCUStore
from pyre_communicator.base_class import PyreBaseCommunicator


class RobotUpdater(PyreBaseCommunicator):

    def __init__(self):
        super().__init__('robot_updater', ['ROPOD', 'ROBOT-UPDATER'], [], verbose=False)


    def setup(self):
        print('Preparing the CCUStore')
        ccu_store = CCUStore('ropod_ccu_store')

        # create two Waypoints (one for each area)
        waypoint_A = Waypoint()
        waypoint_A.semantic_id = '0'
        waypoint_A.area_id = 1
        waypoint_A.x = '1'
        waypoint_A.y = '1'

        waypoint_B = Waypoint()
        waypoint_B.semantic_id = '1'
        waypoint_B.area_id = 2
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

        status_A = RobotStatus()
        status_A.robot_id = 'ropod_A'
        status_A.current_location = area_A
        status_A.current_operation = 'hangout'
        status_A.status = 'idle'
        status_A.available = 'na'
        status_A.battery_status = 'voll Saft'

        ccu_store.add_robot_status(status_A)

        robot_A = Robot()

        robot_A.robot_id = 'ropod_A'
        robot_A.schedule = 'N/A'
        robot_A.status = status_A

        ccu_store.add_robot(robot_A)

        robot_B = robot_A
        robot_B.robot_id = 'ropod_B'
        robot_B.status.robot_id = 'roopd_B'
        ccu_store.add_robot(robot_B)

        # this one will at as a contorl and will NOT be changed
        robot_C = robot_A
        robot_C.robot_id = 'ropod_C'
        robot_C.status.robot_id = 'roopd_C'
        ccu_store.add_robot(robot_C)


    def send_request(self):
        self.setup()
        update_files = ['config/msgs/robot/ropod-location-change_A.json',
                        'config/msgs/robot/ropod-location-change_B.json']

        for update_file in update_files:
            with open(update_file) as json_file:
                robot_update = json.load(json_file)

            robot_update['header']['queryId'] = self.generate_uuid()
            robot_update['header']['timestamp'] = self.get_time_stamp()

            robot_update['payload']['taskId'] = self.generate_uuid()

            print("Sending ROPOD update")
            self.shout(robot_update, "ROPOD")

if __name__ == '__main__':
    test = RobotUpdater()
    time.sleep(5)
    test.send_request()
    time.sleep(1)
    print("Request sent. Check the database for updated location")
    test.shutdown()
