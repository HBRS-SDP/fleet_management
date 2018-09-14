from __future__ import print_function
import time

from pyre_communicator.base_class import PyreBaseCommunicator


class TaskRequester(PyreBaseCommunicator):
    def __init__(self):
        super().__init__('task_request_test', ['ROPOD'], [], verbose=False)
        pass

    def send_request(self):

        task_request_msg = dict()
        task_request_msg['header'] = dict()
        task_request_msg['payload'] = dict()

        task_request_msg['header']['type'] = 'TASK-REQUEST'
        task_request_msg['header']['metamodel'] = 'ropod-msg-schema.json'
        task_request_msg['header']['msgId'] = self.generate_uuid()
        task_request_msg['header']['timestamp'] = self.get_time_stamp()

        task_request_msg['payload']['metamodel'] = 'ropod-task-request-schema.json'
        task_request_msg['payload']['userId'] = '1'
        task_request_msg['payload']['deviceType'] = 'mobidik'
        task_request_msg['payload']['deviceId'] = '4800001663'
        task_request_msg['payload']['pickupLocation'] = 'AMK_D_L-1_C41_LA1'
        task_request_msg['payload']['deliveryLocation'] = 'AMK_B_L4_C1_LA2'
        task_request_msg['payload']['pickupLocationLevel'] = -1
        task_request_msg['payload']['deliveryLocationLevel'] = 4
        task_request_msg['payload']['startTime'] = int(round(time.time()) * 1000) + 10

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
        test.send_request()
        while not test.terminated:
            time.sleep(0.5)
        raise KeyboardInterrupt
    except (KeyboardInterrupt, SystemExit):
        print("Exiting test...")
        test.shutdown()
        print('FMS interrupted; exiting')
