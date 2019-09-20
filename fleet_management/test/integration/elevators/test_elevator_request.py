from __future__ import print_function

import time

from fleet_management.test.fixtures.utils import get_msg_fixture
from ropod.pyre_communicator.base_class import RopodPyre
from ropod.utils.timestamp import TimeStamp
from ropod.utils.uuid import generate_uuid


class ElevatorRequester(RopodPyre):
    def __init__(self):
        zyre_config = {'node_name': 'elevator_requester',
                       'groups': ['ROPOD', 'ELEVATOR-CONTROL'],
                       'message_types': []}
        super().__init__(zyre_config)

    def send_request(self):
        elevator_request = get_msg_fixture('elevator', 'ropod-elevator-request.json')

        elevator_request['header']['queryId'] = generate_uuid()
        elevator_request['header']['timestamp'] = TimeStamp().to_str()

        elevator_request['payload']['taskId'] = generate_uuid()
        elevator_request['payload']['load'] = 'MobiDik'

        start = elevator_request.get('payload').get('startFloor')
        goal = elevator_request.get('payload').get('goalFloor')

        print("Sending elevator request from floor %s to floor %s" % (start, goal))
        self.shout(elevator_request, ["ROPOD"])

    def receive_msg_cb(self, msg_content):
        message = self.convert_zyre_msg_to_dict(msg_content)
        if message is None:
            return

        if message['header']['type'] == 'ROBOT-ELEVATOR-CALL-REPLY':
            print("[INFO] Received elevator confirmation from resource "
                  "manager.")
        elif message['header']['type'] == 'ELEVATOR-STATUS':
            if message['payload']['doorOpenAtStartFloor']:
                time.sleep(1)
                print("Sending confirmation of entering elevator....")
                enter_confirmation_msg = get_msg_fixture('elevator', 'ropod-elevator-enter-confirmation.json')

                self.shout(enter_confirmation_msg, "ROPOD")
            elif message['payload']['doorOpenAtGoalFloor']:
                time.sleep(1)
                print("[INFO] Sending confirmation of exiting elevator....")

                exit_confirmation_msg = get_msg_fixture('elevator', 'ropod-elevator-exit-confirmation.json')

                self.shout(exit_confirmation_msg, ["ROPOD"])
                self.terminated = True


if __name__ == '__main__':
    test = ElevatorRequester()
    test.start()

    try:
        time.sleep(15)
        test.send_request()
        while not test.terminated:
            time.sleep(0.5)
        raise KeyboardInterrupt
    except (KeyboardInterrupt, SystemExit):
        print("Exiting test...")
        test.shutdown()
