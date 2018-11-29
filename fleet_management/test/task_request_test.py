from __future__ import print_function
import time
import json

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

        task_request_msg['payload']['startTime'] = self.get_time_stamp()

        print("Sending task request")
        print(task_request_msg)
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
        time.sleep(5)
        test.send_request()
        while not test.terminated:
            time.sleep(0.5)
        raise KeyboardInterrupt
    except (KeyboardInterrupt, SystemExit):
        print("Exiting test...")
        test.shutdown()
        print('Task request test interrupted; exiting')
