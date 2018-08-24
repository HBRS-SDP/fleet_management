from __future__ import print_function
import uuid
import time

from pyre_communicator.base_class import PyreBaseCommunicator

if __name__ == '__main__':
    task_request_msg = dict()
    task_request_msg['header'] = dict()
    task_request_msg['payload'] = dict()

    task_request_msg['header']['type'] = 'TASK-REQUEST'
    task_request_msg['header']['metamodel'] = 'ropod-msg-schema.json'
    task_request_msg['header']['msgId'] = str(uuid.uuid4())
    task_request_msg['header']['timestamp'] = int(round(time.time()) * 1000)

    task_request_msg['payload']['metamodel'] = 'ropod-task-request-schema.json'
    task_request_msg['payload']['userId'] = '1'
    task_request_msg['payload']['deviceType'] = 'mobidik'
    task_request_msg['payload']['deviceId'] = '4800001663'
    task_request_msg['payload']['pickupLocation'] = 'AMK_B_L-1_C41_LA2'
    task_request_msg['payload']['deliveryLocation'] = 'AMK_B_L4_C1_LA2'
    task_request_msg['payload']['pickupLocationLevel'] = -1
    task_request_msg['payload']['deliveryLocationLevel'] = 4
    task_request_msg['payload']['startTime'] = int(round(time.time()) * 1000) + 10

    pyre_communicator = PyreBaseCommunicator('task_request_test', ['ROPOD'], [], verbose=True)
    pyre_communicator.shout(task_request_msg)
