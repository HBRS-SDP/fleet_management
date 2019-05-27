from __future__ import print_function
import time
import json
import sys
from datetime import timedelta

from ropod.pyre_communicator.base_class import RopodPyre
from ropod.utils.uuid import generate_uuid
from ropod.utils.timestamp import TimeStamp as ts


class TaskRequester(RopodPyre):
    def __init__(self):
        super().__init__('task_request_test', ['ROPOD'], ['TASK-REQUEST'], verbose=True, acknowledge=True)

    def send_request(self, config_file):
        """ Send task request to fleet management system via pyre

        :config_file: string (path to the config file containing task request
        :returns: None

        """
        print("Preparing task request message")
        with open(config_file) as json_file:
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
    if len(sys.argv) > 1 and sys.argv[1] == "invalid":
        config_file = 'config/msgs/task_requests/task-request-mobidik-invalid.json'
    else:
        config_file = 'config/msgs/task_requests/task-request-mobidik.json'

    timeout_duration = 300 # 5 minutes

    test = TaskRequester()
    test.start()

    try:
        time.sleep(10)
        test.send_request(config_file)
        # TODO: receive msg from ccu for invalid task request instead of timeout
        start_time = time.time()
        while not test.terminated and start_time + timeout_duration > time.time():
            time.sleep(0.5)
    except (KeyboardInterrupt, SystemExit):
        print('Task request test interrupted; exiting')

    print("Exiting test...")
    test.shutdown()
