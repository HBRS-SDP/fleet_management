from __future__ import print_function

import json
import sys
import time
import unittest

from ropod.pyre_communicator.base_class import RopodPyre
from ropod.structs.elevator import Elevator
from ropod.utils.timestamp import TimeStamp
from ropod.utils.uuid import generate_uuid

from fleet_management.db.ccu_store import CCUStore

VERBOSE = False


class ElevatorUpdater(RopodPyre):

    def __init__(self):
        zyre_config = {'node_name': 'elevator_updater',
                       'groups': ['ROPOD', 'ELEVATOR-UPDATER'],
                       'message_types': []}
        super().__init__(zyre_config)
        print('Preparing the CCUStore')
        self.ccu_store = CCUStore('ropod_ccu_store')
        self.verification = {}

    def setup(self):
        # create and add some elevators
        elevator_A = Elevator(65)
        elevator_A.floor = 0
        elevator_A.calls = 0
        elevator_A.is_available = True
        elevator_A.door_open_at_goal_floor = False
        elevator_A.door_open_at_start_floor = False

        self.ccu_store.add_elevator(elevator_A)
        self.ccu_store.update_elevator(elevator_A)

        elevator_B = elevator_A
        elevator_B.elevator_id = 66
        self.ccu_store.add_elevator(elevator_B)
        self.ccu_store.update_elevator(elevator_B)

        elevator_C = elevator_A
        elevator_C.elevator_id = 67
        self.ccu_store.add_elevator(elevator_C)
        self.ccu_store.update_elevator(elevator_C)

    def send_request(self):
        self.setup()
        update_files = ['config/msgs/elevator/ropod-elevator-change_A.json',
                        'config/msgs/elevator/ropod-elevator-change_B.json']

        for update_file in update_files:
            with open(update_file) as json_file:
                elevator_update = json.load(json_file)

            elevator_update['header']['queryId'] = generate_uuid()
            elevator_update['header']['timestamp'] = TimeStamp().to_str()

            elevator_update['payload']['taskId'] = generate_uuid()

            self.verification[elevator_update['payload']['elevatorId']] \
                = elevator_update

            self.shout(elevator_update, "ROPOD")

    # get all of the elevators from the ccu store and make sure they are
    # actually updated compared to what we are expecting
    def verify(self):
        success = True
        elevators = self.ccu_store.get_elevators()

        for key, value in self.verification.items():
            # we are only going to compare on a few things:
            #   floor, calls, & is_available

            # it's possible this will through an error but we don't need to
            # catch it because if this test fails then something is already
            # wrong.
            actual_elevator = elevators[key]

            success = actual_elevator.floor == value['payload']['floor'] \
                      and actual_elevator.calls == value['payload']['calls'] \
                      and actual_elevator.is_available \
                      == value['payload']['isAvailable']
            if VERBOSE:
                print("Success for ", key, "was", success)

        return success


class TestElevatorUpdater(unittest.TestCase):

    def setUp(self):
        wait_seconds = 16
        self.updater = ElevatorUpdater()
        self.updater.start()
        print("Please wait ", wait_seconds, " before the test will begin.")
        time.sleep(wait_seconds)

        self.updater.send_request()
        if VERBOSE:
            print("Request sent.")
        time.sleep(1)

    def test_insertAndCheck(self):
        if VERBOSE:
            print("Attempting to verify...")
        success = self.updater.verify()
        self.assertTrue(success)

        if VERBOSE:
            print("\nThe test was a: ")
            if success:
                print("SUCCESS")
            else:
                print("FAILURE")
            print("\nRegardless, you should still check the database manually to be \
                    s a f e")

    def tearDown(self):
        self.updater.shutdown()


if __name__ == '__main__':
    suite = unittest.TestLoader().loadTestsFromTestCase(TestElevatorUpdater)
    res = unittest.TextTestRunner(verbosity=2).run(suite)

    exit_value = 0
    if not res.wasSuccessful():
        exit_value = 1

    sys.exit(exit_value)
