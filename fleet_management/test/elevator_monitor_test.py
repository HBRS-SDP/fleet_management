from __future__ import print_function
import time
import json
import sys

from ropod.structs.elevator import Elevator
from fleet_management.db.ccu_store import CCUStore
from ropod.pyre_communicator.base_class import PyreBaseCommunicator


class ElevatorUpdater(PyreBaseCommunicator):

    def __init__(self):
        super().__init__('elevator_updater', ['ROPOD', 'ELEVATOR-UPDATER'], [], verbose=False)
        print('Preparing the CCUStore')
        self.ccu_store = CCUStore('ropod_ccu_store')
        self.verification = {}

    def setup(self):
        # create and add some elevators
        elevator_A = Elevator()
        elevator_A.elevator_id = 65
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

            elevator_update['header']['queryId'] = self.generate_uuid()
            elevator_update['header']['timestamp'] = self.get_time_stamp()

            elevator_update['payload']['taskId'] = self.generate_uuid()

            self.verification[elevator_update['payload']['id']] \
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

            # print(actual_elevator.floor, value['payload']['floor'], \
            #      actual_elevator.calls, value['payload']['calls'], \
            #      actual_elevator.is_available, \
            #          value['payload']['isAvailable'])

            success = actual_elevator.floor == value['payload']['floor'] \
                      and actual_elevator.calls == value['payload']['calls'] \
                      and actual_elevator.is_available \
                      == value['payload']['isAvailable']

            print("Success for ", key, "was", success)

        return success


if __name__ == '__main__':
    wait_seconds = 16
    exit_code = 0
    test = ElevatorUpdater()

    print("Please wait ", wait_seconds, " before the test will begin.")
    time.sleep(wait_seconds)
    test.send_request()
    print("Request sent.")
    time.sleep(1)

    print("Attempting to verify...")
    success = test.verify()
    print("\nThe test was a: ")
    if success:
        print("SUCCESS")
    else:
        print("FAILURE")
        exit_code = 1
    print("\nRegardless, you should still check the database manually to be \
            s a f e")

    test.shutdown()
    sys.exit(exit_code)
