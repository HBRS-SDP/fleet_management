from __future__ import print_function
import time
import json
from datetime import timedelta

from ropod.pyre_communicator.base_class import RopodPyre
from ropod.utils.uuid import generate_uuid
from ropod.utils.timestamp import TimeStamp as ts


class TaskRequester(RopodPyre):
    def __init__(self):
        zyre_config = {'node_name': 'task_request_test',
                       'groups': ['ROPOD'],
                       'message_types': ['TASK-REQUEST']}
        super().__init__(zyre_config, acknowledge=True)

    def send_request(self):
        print("Preparing task request message")
        with open('config/msgs/task_requests/task-request-mobidik.json') as json_file:
            task_request_msg = json.load(json_file)

        task_request_msg['header']['msgId'] = generate_uuid()
        task_request_msg['header']['timestamp'] = ts.get_time_stamp()

        delta = timedelta(minutes=1)

        task_request_msg['payload']['earliestStartTime'] = ts.get_time_stamp(delta)

        delta = timedelta(minutes=1, seconds=30)

        task_request_msg['payload']['latestStartTime'] = ts.get_time_stamp(delta)

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
    test.start()

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
