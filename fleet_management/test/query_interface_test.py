from __future__ import print_function
import time
import os.path
import json

from ropod.pyre_communicator.base_class import RopodPyre
from ropod.utils.models import MessageFactory
from ropod.utils.uuid import generate_uuid

class QueryTest(RopodPyre):
    def __init__(self):
        super(QueryTest, self).__init__('ccu_query_test', ['ROPOD'], [], verbose=False, acknowledge=True)
        self.start()

        self.num_of_tests = 0
        self.num_of_responses = 0
        self.num_of_success = 0

        random_task_id = generate_uuid()
        robot_id = 'ropod_001'

        self.send_request("GET-ALL-ONGOING-TASKS")
        self.send_request("GET-ALL-SCHEDULED-TASKS")
        self.send_request("GET-ALL-SCHEDULED-TASK-IDS")
        self.send_request("GET-ROBOTS-ASSIGNED-TO-TASK", {'taskId':random_task_id})
        self.send_request("GET-TASKS-ASSIGNED-TO-ROBOT", {'robotId': robot_id})
        self.send_request("GET-ROBOT-STATUS", {'robotId': robot_id})

    def send_request(self, msg_type, payload_dict=None):
        time.sleep(1)
        self.num_of_tests += 1

        query_msg = MessageFactory.get_header(msg_type, recipients=[])

        query_msg['payload'] = {}
        query_msg['payload']['senderId'] = generate_uuid()
        if payload_dict is not None :
            for key in payload_dict.keys() :
                query_msg['payload'][key] = payload_dict[key]

        # query_msg = json.dumps(query_msg, indent=2, default=str)

        self.shout(query_msg)

    def receive_msg_cb(self, msg_content):
        message = self.convert_zyre_msg_to_dict(msg_content)
        if message is None:
            return

        if message['header']['type'] in ["GET-ALL-ONGOING-TASKS", 
                "GET-ALL-SCHEDULED-TASKS", "GET-TASKS-ASSIGNED-TO-ROBOT"] :
            self.contains_key_in_payload('tasks', message)

        elif message['header']['type'] in ['GET-ROBOTS-ASSIGNED-TO-TASK']:
            self.contains_key_in_payload('robots', message)

        elif message['header']['type'] in ['GET-ALL-SCHEDULED-TASK-IDS']:
            self.contains_key_in_payload('taskIds', message)
            task_ids = message['payload']['taskIds']
            if task_ids :
                self.send_request("GET-ROBOTS-ASSIGNED-TO-TASK", {'taskId':task_ids[0]})

        elif message['header']['type'] == "GET-ROBOT-STATUS":
            self.contains_key_in_payload('status', message)

        print(message['payload']['success'])
        self.num_of_responses += 1

    def contains_key_in_payload(self, key, message) :
        try:
            assert key in message['payload'].keys()
            self.num_of_success += 1
            print(message['header']['type'], "Test passed")
        except Exception as e:
            print(message['header']['type'], "Test failed")


if __name__ == '__main__':
    test = QueryTest()

    try:
        n = 0
        while test.num_of_tests > test.num_of_responses and n < 10 :
            n += 1
            time.sleep(0.5)
    except (KeyboardInterrupt, SystemExit):
        print("Exiting test...")
        test.shutdown()
        print('Task request test interrupted; exiting')
    test.shutdown()
    print(test.num_of_success, "tests passed out of", test.num_of_tests)
