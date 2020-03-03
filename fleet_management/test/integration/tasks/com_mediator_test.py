from __future__ import print_function

import os
import time
import uuid
import unittest
import pymongo as pm
import yaml
import argparse

from ropod.pyre_communicator.base_class import RopodPyre
from ropod.utils.models import RopodMessageFactory
from ropod.utils.uuid import generate_uuid
from ropod.structs.status import TaskStatus

from fleet_management.config.loader import default_config
from fleet_management.api.interfaces.query import QueryInterface

from fmlib.models.tasks import Task
from fmlib.models.robot import Robot
from fmlib.models.requests import TaskRequest
from fmlib.db.mongo import MongoStore
from fmlib.utils.utils import load_file_from_module, load_yaml
from fmlib.utils.messages import Message
from fleet_management.test.fixtures.utils import get_msg_fixture


class QueryTest(RopodPyre):
    def __init__(self):
        zyre_config = {'node_name': 'fms_query_test',
                       'groups': ['ROPOD'],
                       'message_types': []}
        super(QueryTest, self).__init__(zyre_config)
        self.response = None
        self.start()

    def send_request(self, msg_type, payload_dict=None):
        query_msg = RopodMessageFactory.get_header(msg_type, recipients=[])

        query_msg['payload'] = {}
        query_msg['payload']['senderId'] = generate_uuid()
        if payload_dict is not None:
            for key in payload_dict.keys():
                query_msg['payload'][key] = payload_dict[key]

        # print(json.dumps(query_msg, indent=2, default=str))
        self.shout(query_msg)

    def receive_msg_cb(self, msg_content):
        message = self.convert_zyre_msg_to_dict(msg_content)
        if message is None:
            return

        # Ignore messages with type other than TASK or TASK_STATUS
        if message['header']['type'] != "TASK" and \
            message['header']['type'] != "TASK-STATUS":
            #print("Ignoring message of type " + message['header']['type'])
            return

        self.response = message


class QueryInterfaceTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.test_pyre_node = QueryTest()
        cls.timeout_duration = 3
        time.sleep(3)
        cls.test_result_successfull = []

    @classmethod
    def tearDownClass(cls):
        cls.test_pyre_node.shutdown()
        # print(cls.test_result_successfull)
        if all(cls.test_result_successfull):
            print()
            print('-'*80)
            print('Ran', len(cls.test_result_successfull), 'tests\n')
            print("OK")
            os._exit(0)
        else:
            os._exit(1)

    def setUp(self):
        pass

    def tearDown(self):
        if hasattr(self, '_outcome'):
            result = self.defaultTestResult()
            self._feedErrorsToResult(result, self._outcome.errors)
            self.test_result_successfull.append(result.wasSuccessful())
            if not result.wasSuccessful():
                for failure in result.failures:
                    print(failure[0])
                    print(failure[1])
                for error in result.errors:
                    print(error[0])
                    print(error[1])
        else:
            self.test_result_successfull.append(True)

    def test_task_status_response(self):
        # Validate TASK message and get taskId and actionId
        message = self.wait_for_task_msg()
        self.assertNotEqual(message, None)
        self.assertIn('header', message)
        self.assertIn('payload', message)
        self.assertIn('type', message['header'])
        self.assertEqual(message['header']['type'], "TASK")
        self.assertIn('taskId', message['payload'])
        self.assertIn('plan', message['payload'])
        self.assertGreater(len(message['payload']['plan']), 0)
        self.assertIn('actions', message['payload']['plan'][0])
        self.assertGreater(len(message['payload']['plan'][0]['actions']), 0)
        self.assertIn('_id', message['payload']['plan'][0]['actions'][0])
        self.assertIn('type', message['payload']['plan'][0]['actions'][0])
        task_id = message['payload']['taskId']
        action_id = message['payload']['plan'][0]['actions'][0]['_id']
        action_type = message['payload']['plan'][0]['actions'][0]['type']

        # Validate TASK-STATUS message and verify task Id
        message = self.wait_for_task_status_msg()
        self.assertNotEqual(message, None)
        self.assertIn('header', message)
        self.assertIn('payload', message)
        self.assertIn('type', message['header'])
        self.assertEqual(message['header']['type'], "TASK-STATUS")
        self.assertIn('taskId', message['payload'])
        self.assertIn('taskProgress', message['payload'])
        self.assertIn('actionId', message['payload']['taskProgress'])
        self.assertIn('actionType', message['payload']['taskProgress'])
        self.assertEqual(task_id, message['payload']['taskId'])
        self.assertEqual(action_id, message['payload']['taskProgress']['actionId'])
        self.assertEqual(action_type, message['payload']['taskProgress']['actionType'])

    def wait_for_task_msg(self, wait_time=120):
        print("Waiting {0} seconds for a task message".format(wait_time))
        start_time = time.time()
        while self.test_pyre_node.response is None and \
                start_time + wait_time > time.time():
            time.sleep(0.2)
        message = self.test_pyre_node.response

        if message is None:
            print("Wait for task message timed out!")
        else:
            print("Received a task message")

        self.test_pyre_node.response = None
        return message

    def wait_for_task_status_msg(self, wait_time=30):
        print("Waiting {0} seconds for a task status message".format(wait_time))
        start_time = time.time()
        while self.test_pyre_node.response is None and \
                start_time + wait_time > time.time():
            time.sleep(0.2)
        message = self.test_pyre_node.response

        if message is None:
            print("Wait for task status message timed out!")
        else:
            print("Received a task status message")

        self.test_pyre_node.response = None
        return message

if __name__ == '__main__':
    unittest.main()
