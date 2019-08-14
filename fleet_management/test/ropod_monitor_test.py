from __future__ import print_function

import json
import sys
import time
import unittest

from ropod.pyre_communicator.base_class import RopodPyre
from ropod.structs.area import Area
from ropod.structs.area import SubArea
from ropod.structs.robot import Robot
from ropod.structs.status import RobotStatus
from ropod.utils.timestamp import TimeStamp as ts
from ropod.utils.uuid import generate_uuid

from fleet_management.db.ccu_store import CCUStore

VERBOSE = False


class RobotUpdater(RopodPyre):

    def __init__(self):
        zyre_config = {'node_name': 'robot_updater',
                       'groups': ['ROPOD', 'ROBOT-UPDATER'],
                       'message_types': []}
        super().__init__(zyre_config)
        print('Preparing the CCUStore')
        self.ccu_store = CCUStore('ropod_ccu_store')
        self.verification = {}

    def setup(self):
        # create two SubArea (one for each area)
        waypoint_A = SubArea()
        waypoint_A.semantic_id = '0'
        waypoint_A.area_id = 1
        waypoint_A.x = '1'
        waypoint_A.y = '1'

        waypoint_B = SubArea()
        waypoint_B.semantic_id = '1'
        waypoint_B.area_id = 2
        waypoint_B.x = '2'
        waypoint_B.y = '2'

        # create an area for each one of our SubArea
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

        robot_001 = Robot('ropod_001')

        robot_001.schedule = 'N/A'
        robot_001.status = status_001

        self.ccu_store.add_robot(robot_001)
        if VERBOSE:
            print("Added robot 001")

        robot_002 = robot_001
        robot_002.robot_id = 'ropod_002'
        robot_002.status.robot_id = 'ropod_002'
        self.ccu_store.add_robot(robot_002)
        if VERBOSE:
            print("Added robot 002")

        # this one will at as a contorl and will NOT be changed
        robot_003 = robot_001
        robot_003.robot_id = 'ropod_003'
        robot_003.status.robot_id = 'roopd_003'
        self.ccu_store.add_robot(robot_003)
        if VERBOSE:
            print("Added robot 003")

    def send_request(self):
        self.setup()
        update_files = ['config/msgs/robot/ropod-location-change_A.json',
                        'config/msgs/robot/ropod-location-change_B.json']

        for update_file in update_files:
            with open(update_file) as json_file:
                robot_update = json.load(json_file)

            robot_update['header']['queryId'] = generate_uuid()
            robot_update['header']['timestamp'] = ts.get_time_stamp()

            robot_update['payload']['taskId'] = generate_uuid()

            self.verification[robot_update['payload']['robotId']] \
                = robot_update

            self.shout(robot_update, "ROPOD")

    # get all of the robots from the ccu store and make sure they are
    # actually updated compared to what we are expecting
    def verify(self):
        success = True
        robots = self.ccu_store.get_robots()

        for key, value in self.verification.items():
            if VERBOSE:
                print("\n", key, value)
            # we are only going to compare on a few things: (spot check)
            #   currentOperation, currentLocation[name, floorNumber]

            # it's possible this will throw an error but we don't need to
            # catch it because if this test fails then something is already
            # wrong.
            actual_robot = robots[key]
            actual_status = actual_robot.status
            actual_area = actual_status.current_location

            if VERBOSE:
                print("Actual vs Verification")
                print(actual_status.current_operation,
                      value['payload']['currentOperation'])
                print(actual_area.name,
                      value['payload']['currentLocation']['name'])
                print(actual_area.floor_number,
                      value['payload']['currentLocation']['floorNumber'])

            success = actual_status.current_operation == \
                      value['payload']['currentOperation'] \
                      and actual_area.name == \
                      value['payload']['currentLocation']['name'] \
                      and actual_area.floor_number == \
                      value['payload']['currentLocation']['floorNumber']

            if VERBOSE:
                print("Success for ", key, "was", success, "\n")

        return success


class TestRobotUpdater(unittest.TestCase):

    def setUp(self):
        wait_seconds = 15
        exit_code = 0
        self.updater = RobotUpdater()
        self.updater.start()

        print("Please wait ", wait_seconds, " seconds before the test will begin.")
        time.sleep(wait_seconds)
        self.updater.send_request()
        if VERBOSE:
            print("Request sent.")
        time.sleep(1)

    def test_insertAndCheck(self):
        if VERBOSE:
            print("\nAttempting to verify...")
        success = self.updater.verify()
        self.assertTrue(success)

        if VERBOSE:
            print("\nThe test was a: ")
            if success:
                print("SUCCESS")
            else:
                print("FAILURE")
            print("\nRegardless you should still check the database manually to be safe!")

    def tearDown(self):
        self.updater.shutdown()


if __name__ == '__main__':
    suite = unittest.TestLoader().loadTestsFromTestCase(TestRobotUpdater)
    res = unittest.TextTestRunner(verbosity=2).run(suite)

    exit_value = 0
    if not res.wasSuccessful():
        exit_value = 1

    print("eval: " + str(exit_value))
    sys.exit(exit_value)
