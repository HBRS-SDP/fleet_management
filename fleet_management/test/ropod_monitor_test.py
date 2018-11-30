from __future__ import print_function
import time
import json
import sys

from fleet_management.structs.robot import Robot
from fleet_management.structs.area import Area
from fleet_management.structs.area import Waypoint
from fleet_management.structs.status import RobotStatus
from fleet_management.db.ccu_store import CCUStore
from ropod.pyre_communicator.base_class import PyreBaseCommunicator


class RobotUpdater(PyreBaseCommunicator):

    def __init__(self):
        super().__init__('robot_updater', ['ROPOD', 'ROBOT-UPDATER'], [], verbose=False)
        print('Preparing the CCUStore')
        self.ccu_store = CCUStore('ropod_ccu_store')
        self.verification = {}


    def setup(self):
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

        status_001 = RobotStatus()
        status_001.robot_id = 'ropod_001'
        status_001.current_location = area_A
        status_001.current_operation = 'hangout'
        status_001.status = 'idle'
        status_001.available = 'na'
        status_001.battery_status = 'voll Saft'

        self.ccu_store.add_robot_status(status_001)

        robot_001 = Robot()

        robot_001.robot_id = 'ropod_001'
        robot_001.schedule = 'N/A'
        robot_001.status = status_001

        self.ccu_store.add_robot(robot_001)
        print("Added robot 001")

        robot_002 = robot_001
        robot_002.robot_id = 'ropod_002'
        robot_002.status.robot_id = 'ropod_002'
        self.ccu_store.add_robot(robot_002)
        print("Added robot 002")

        # this one will at as a contorl and will NOT be changed
        robot_003 = robot_001
        robot_003.robot_id = 'ropod_003'
        robot_003.status.robot_id = 'roopd_003'
        self.ccu_store.add_robot(robot_003)
        print("Added robot 003")


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

            self.verification[robot_update['payload']['robotId']] \
                    = robot_update

            self.shout(robot_update, "ROPOD")


    # get all of the robots from the ccu store and make sure they are
    # actually updated compared to what we are expecting
    def verify(self):
        success = True
        robots = self.ccu_store.get_robots()

        for key, value in self.verification.items():
            print("\n", key, value)
            # we are only going to compare on a few things: (spot check)
            #   currentOperation, currentLocation[name, floorNumber]

            # it's possible this will throw an error but we don't need to
            # catch it because if this test fails then something is already
            # wrong.
            actual_robot = robots[key]
            actual_status = actual_robot.status
            actual_area = actual_status.current_location

            print("Actual vs Verification")
            print(actual_status.current_operation, \
                value['payload']['currentOperation'])
            print(actual_area.name,
                value['payload']['currentLocation']['name'])
            print(actual_area.floor_number, \
                value['payload']['currentLocation']['floorNumber'])

            success = actual_status.current_operation == \
                        value['payload']['currentOperation'] \
                  and actual_area.name == \
                        value['payload']['currentLocation']['name'] \
                  and actual_area.floor_number == \
                        value['payload']['currentLocation']['floorNumber']

            print("Success for ", key, "was", success, "\n")

        return success


if __name__ == '__main__':
    wait_seconds = 1
    exit_code = 0
    test = RobotUpdater()

    print("Please wait ", wait_seconds, " seconds before the test will begin.")
    time.sleep(wait_seconds)
    test.send_request()
    print("Request sent.")
    time.sleep(1)

    print("\nAttempting to verify...")
    success = test.verify()
    print("\nThe test was a: ")
    if success:
        print("SUCCESS")
    else:
        print("FAILURE")
        exit_code = 1
    print("\nRegardless you should still check the database manually to be safe!")

    test.shutdown()
    sys.exit(exit_code)
