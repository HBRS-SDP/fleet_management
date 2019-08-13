from __future__ import print_function

import json
import time

from ropod.pyre_communicator.base_class import RopodPyre
from ropod.utils.timestamp import TimeStamp as ts
from ropod.utils.uuid import generate_uuid


class ResourceManagerMockup(RopodPyre):
    def __init__(self):
        super().__init__('resource_monitor_mockup', ['ROPOD'], [], verbose=False)

    def receive_msg_cb(self, msg_content):
        message = self.convert_zyre_msg_to_dict(msg_content)
        if message is None:
            return

        if message['header']['type'] == 'ROBOT-ELEVATOR-CALL-REQUEST':
            with open('config/msgs/elevator/ropod-elevator-call-reply.json') as msg_file:
                elevator_reply_msg = json.load(msg_file)
                elevator_reply_msg['header']['msgId'] = generate_uuid()
                elevator_reply_msg['header']['timestamp'] = ts.get_time_stamp()
                elevator_reply_msg['payload']['queryId'] = message['payload']['queryId']
                self.shout(elevator_reply_msg, "ROPOD")

if __name__ == '__main__':
    mockup = ResourceManagerMockup()
    mockup.start()

    try:
        while not mockup.terminated:
            time.sleep(0.5)
        raise KeyboardInterrupt
    except (KeyboardInterrupt, SystemExit):
        print("Exiting mockup...")
        mockup.shutdown()
