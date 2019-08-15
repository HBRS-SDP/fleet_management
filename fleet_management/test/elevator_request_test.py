from __future__ import print_function

import json
import time

from ropod.pyre_communicator.base_class import RopodPyre
from ropod.utils.timestamp import TimeStamp as ts
from ropod.utils.uuid import generate_uuid


class ElevatorRequester(RopodPyre):
    def __init__(self):
        zyre_config = {'node_name': 'elevator_requester',
                       'groups': ['ROPOD', 'ELEVATOR-CONTROL'],
                       'message_types': []}
        super().__init__(zyre_config)

    def send_request(self):
        with open('config/msgs/elevator/ropod-elevator-request.json') as json_file:
            elevator_request = json.load(json_file)

        elevator_request['header']['queryId'] = generate_uuid()
        elevator_request['header']['timestamp'] = ts.get_time_stamp()

        elevator_request['payload']['taskId'] = generate_uuid()
        elevator_request['payload']['load'] = 'MobiDik'

        start = elevator_request.get('payload').get('startFloor')
        goal = elevator_request.get('payload').get('goalFloor')

        print("Sending elevator request from floor %s to floor %s" % (start, goal))
        self.shout(elevator_request, "ROPOD")

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
                with open('config/msgs/elevator/ropod-elevator-enter-confirmation.json') as msg_file:
                    enter_confirmation_msg = json.load(msg_file)

                self.shout(enter_confirmation_msg, "ROPOD")
            elif message['payload']['doorOpenAtGoalFloor']:
                time.sleep(1)
                print("[INFO] Sending confirmation of exiting elevator....")

                with open("config/msgs/elevator/ropod-elevator-exit-confirmation.json") as msg_file:
                    exit_confirmation_msg = json.load(msg_file)

                self.shout(exit_confirmation_msg, "ROPOD")
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
