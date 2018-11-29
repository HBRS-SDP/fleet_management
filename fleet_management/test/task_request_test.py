from __future__ import print_function
import time
import json
from datetime import timedelta, datetime
from fleet_management.db.ccu_store import CCUStore
from fleet_management.structs.area import Area
from fleet_management.structs.area import Waypoint
from fleet_management.structs.status import RobotStatus
from fleet_management.structs.robot import Robot


from ropod.pyre_communicator.base_class import PyreBaseCommunicator


class TaskRequester(PyreBaseCommunicator):
    def __init__(self):
        super().__init__('task_request_test', ['ROPOD'], [], verbose=True)

    def send_request(self):
        print("Preparing task request message")
        with open('config/msgs/task_requests/task-request-mobidik.json') as json_file:
            task_request_msg = json.load(json_file)

        task_request_msg['header']['msgId'] = self.generate_uuid()
        task_request_msg['header']['timestamp'] = self.get_time_stamp()

        delta = timedelta(minutes=1)

        task_request_msg['payload']['earliestStartTime'] = self.get_time_stamp(delta)

        delta = timedelta(minutes=1, seconds=30)

        task_request_msg['payload']['latestStartTime'] = self.get_time_stamp(delta)

        print("Sending task request")
        self.shout(task_request_msg)

    def receive_msg_cb(self, msg_content):
        message = self.convert_zyre_msg_to_dict(msg_content)
        if message is None:
            return

        if message['header']['type'] == 'TASK':
            print("Received task message")
            self.terminated = True


if __name__ == '__main__':
    test = TaskRequester()
    try:
        time.sleep(10)
        test.send_request()
        while not test.terminated:
            time.sleep(0.5)
        raise KeyboardInterrupt
    except (KeyboardInterrupt, SystemExit):
        print("Exiting test...")
        test.shutdown()
        print('Task request test interrupted; exiting')
