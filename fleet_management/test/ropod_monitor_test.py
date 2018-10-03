from __future__ import print_function
import time

from pyre_communicator.base_class import PyreBaseCommunicator


class RopodUpdater(PyreBaseCommunicator):
    def __init__(self):
        super().__init__('task_request_test', ['ROPOD'], [], verbose=True)
        pass

    def send_request(self):

        ropod_update_msg = dict()
        ropod_update_msg['header'] = dict()
        ropod_update_msg['payload'] = dict()

        ropod_update_msg['header']['type'] = 'TASK-REQUEST'
        ropod_update_msg['header']['metamodel'] = 'ropod-msg-schema.json'
        ropod_update_msg['header']['msgId'] = self.generate_uuid()
        ropod_update_msg['header']['timestamp'] = self.get_time_stamp()

        ropod_update_msg['payload']['metamodel'] = 'ropod-task-request-schema.json'
        ropod_update_msg['payload']['userId'] = '1'
        ropod_update_msg['payload']['deviceType'] = 'mobidik'
        ropod_update_msg['payload']['deviceId'] = '4800001663'
        ropod_update_msg['payload']['pickupLocation'] = 'AMK_D_L-1_C41_LA1'
        ropod_update_msg['payload']['deliveryLocation'] = 'AMK_B_L4_C1_LA2'
        ropod_update_msg['payload']['pickupLocationLevel'] = -1
        ropod_update_msg['payload']['deliveryLocationLevel'] = 4
        ropod_update_msg['payload']['startTime'] = self.get_time_stamp()

        print("Sending task request")
        self.shout(ropod_update_msg)

    def receive_msg_cb(self, msg_content):
        message = self.convert_zyre_msg_to_dict(msg_content)
        if message is None:
            return

        if message['header']['type'] == 'TASK':
            print("Received task message")
            self.terminated = True


if __name__ == '__main__':
    test = RopodUpdater()
    try:
        time.sleep(5)
        test.send_request()
        while not test.terminated:
            time.sleep(0.5)
        raise KeyboardInterrupt
    except (KeyboardInterrupt, SystemExit):
        print("Exiting test...")
        test.shutdown()
        print('FMS interrupted; exiting')
