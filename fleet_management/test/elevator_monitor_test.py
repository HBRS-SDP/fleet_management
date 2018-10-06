from __future__ import print_function
import time
import json

from fleet_management.structs.elevator import Elevator
from fleet_management.db.ccu_store import CCUStore
from pyre_communicator.base_class import PyreBaseCommunicator


class ElevatorUpdater(PyreBaseCommunicator):

    def __init__(self):
        super().__init__('elevator_updater', ['ROPOD', 'ELEVATOR-UPDATER'], [], verbose=False)


    def setup(self):
        print('Preparing the CCUStore')
        ccu_store = CCUStore('ropod_ccu_store')

        # create and add some elevators
        elevator_A = Elevator()
        elevator_A.elevator_id = 65
        elevator_A.floor = 0
        elevator_A.calls = 0
        elevator_A.is_available = True

        ccu_store.add_elevator(elevator_A)

        elevator_B = elevator_A
        elevator_B.elevator_id = 66
        ccu_store.add_elevator(elevator_B)

        elevator_C = elevator_A
        elevator_C.elevator_id = 67
        ccu_store.add_elevator(elevator_C)


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

            self.shout(elevator_update, "ROPOD")

if __name__ == '__main__':
    test = ElevatorUpdater()
    time.sleep(5)
    test.send_request()
    time.sleep(1)
    print("Request sent. Check the database for updated location")
    test.shutdown()
