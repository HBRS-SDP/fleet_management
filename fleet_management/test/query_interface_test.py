from __future__ import print_function
import time
import os.path
import json
from datetime import timedelta

from ropod.pyre_communicator.base_class import RopodPyre
from ropod.utils.uuid import generate_uuid
from ropod.utils.timestamp import TimeStamp as ts


class QueryTest(RopodPyre):
    def __init__(self):
        super(QueryTest, self).__init__('ccu_query_test', ['ROPOD'], [], verbose=False, acknowledge=True)
        self.start()

        self.num_of_tests = 0
        self.num_of_responses = 0
        self.num_of_success = 0

        time.sleep(1)
        self.send_request("query_all_ongoing_task.json")
        time.sleep(1) 
        self.send_request("query_all_scheduled_task.json")

    def send_request(self, file_name):
        self.num_of_tests += 1
        code_dir = os.path.abspath(os.path.dirname(__file__))
        message_file_path = os.path.join(os.path.join(code_dir, "config/msgs/query"), 
                file_name)
        with open(message_file_path) as json_file:
            query_msg = json.load(json_file)

        query_msg['header']['msgId'] = generate_uuid()
        query_msg['header']['timestamp'] = ts.get_time_stamp()

        # print("Sending query")
        self.shout(query_msg)

    def receive_msg_cb(self, msg_content):
        message = self.convert_zyre_msg_to_dict(msg_content)
        if message is None:
            return

        # print(message)
        if message['header']['type'] in ["QUERY-ALL-ONGOING-TASKS", "QUERY-ALL-SCHEDULED-TASKS"] :
            try:
                assert "tasks" in message['payload'].keys()
                self.num_of_success += 1
                print(message['header']['type'], "Test passed")
            except Exception as e:
                print(message['header']['type'], "Test failed")

        self.num_of_responses += 1


if __name__ == '__main__':
    test = QueryTest()

    try:
        while test.num_of_tests > test.num_of_responses :
            time.sleep(0.5)
    except (KeyboardInterrupt, SystemExit):
        print("Exiting test...")
        test.shutdown()
        print('Task request test interrupted; exiting')
    test.shutdown()
    print(test.num_of_success, "tests passed out of", test.num_of_tests)
