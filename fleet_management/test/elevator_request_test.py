from __future__ import print_function
import time
import json

from pyre_communicator.base_class import PyreBaseCommunicator


class ElevatorRequester(PyreBaseCommunicator):
    def __init__(self):
        super().__init__('elevator_requester', ['ROPOD', 'ELEVATOR-CONTROL'], [], verbose=False)

    def send_request(self):
        with open('config/msgs/elevator/ropod-elevator-request.json') as json_file:
            elevator_request = json.load(json_file)

        elevator_request['header']['queryId'] = self.generate_uuid()
        elevator_request['header']['timestamp'] = self.get_time_stamp()

        elevator_request['payload']['taskId'] = self.generate_uuid()

        print("Sending elevator request")
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
                time.sleep(2)
                print("Sending confirmation of entering elevator....")
                with open('config/msgs/elevator/ropod-elevator-enter-confirmation.json') as msg_file:
                    enter_confirmation_msg = json.load(msg_file)

                self.shout(enter_confirmation_msg, "ROPOD")
            elif message['payload']['doorOpenAtGoalFloor']:
                time.sleep(2)
                print("[INFO] Sending confirmation of exiting elevator....")

                with open("config/msgs/elevator/ropod-elevator-exit-confirmation.json") as msg_file:
                    exit_confirmation_msg = json.load(msg_file)

                self.shout(exit_confirmation_msg, "ROPOD")
                self.terminated = True


if __name__ == '__main__':
    test = ElevatorRequester()
    try:
        time.sleep(10)
        test.send_request()
        while not test.terminated:
            time.sleep(0.5)
        raise KeyboardInterrupt
    except (KeyboardInterrupt, SystemExit):
        print("Exiting test...")
        test.shutdown()
