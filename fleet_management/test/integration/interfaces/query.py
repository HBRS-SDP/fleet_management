from __future__ import print_function

import os
import time
import uuid
import unittest
import pymongo as pm
import yaml

from ropod.pyre_communicator.base_class import RopodPyre
from ropod.utils.models import RopodMessageFactory
from ropod.utils.uuid import generate_uuid
from ropod.structs.status import TaskStatus

from fleet_management.config.loader import default_config
from fleet_management.api.interfaces.query import QueryInterface

from fleet_management.db.models.task import TransportationTask as Task
from fleet_management.db.models.robot import Ropod as Robot
from fmlib.models.requests import TaskRequest
from fmlib.db.mongo import MongoStore


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

        self.response = message


class QueryInterfaceTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        db_name = 'ropod_ccu_mock_store'
        cls.create_mock_db(cls, filename='query.yaml', db_name=db_name)

        zyre_config = {'node_name': 'ccu_query_interface',
                       'groups': ['ROPOD'],
                       'message_types': list()}

        cls.query_interface = QueryInterface(zyre_config, db_name)

        cls.test_pyre_node = QueryTest()
        cls.timeout_duration = 3
        time.sleep(3)
        cls.test_result_successfull = []

    @classmethod
    def tearDownClass(cls):
        cls.query_interface.shutdown()
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

    def test_scheduled_tasks(self):
        msg_type = "GET-ALL-SCHEDULED-TASKS"
        message = self.send_request_get_response(msg_type)

        self.assertNotEqual(message, None)
        self.assertIn('header', message)
        self.assertIn('type', message['header'])
        self.assertEqual(message['header']['type'], msg_type)
        self.assertIn('payload', message)
        self.assertIn('tasks', message['payload'])
        self.assertTrue(message['payload']['success'])
        self.assertGreater(len(message['payload']['tasks']), 0)

    def test_ongoing_tasks(self):
        msg_type = "GET-ALL-ONGOING-TASKS"
        message = self.send_request_get_response(msg_type)

        self.assertNotEqual(message, None)
        self.assertIn('header', message)
        self.assertIn('type', message['header'])
        self.assertEqual(message['header']['type'], msg_type)
        self.assertIn('payload', message)
        self.assertIn('tasks', message['payload'])
        self.assertTrue(message['payload']['success'])
        self.assertGreater(len(message['payload']['tasks']), 0)

    def test_scheduled_tasks_ids(self):
        msg_type = "GET-ALL-SCHEDULED-TASK-IDS"
        message = self.send_request_get_response(msg_type)

        self.assertNotEqual(message, None)
        self.assertIn('header', message)
        self.assertIn('type', message['header'])
        self.assertEqual(message['header']['type'], msg_type)
        self.assertIn('payload', message)
        self.assertIn('taskIds', message['payload'])
        self.assertTrue(message['payload']['success'])
        self.assertGreater(len(message['payload']['taskIds']), 0)


    def test_robot_assigned_to_task_negative(self):
        random_task_id = generate_uuid()
        robot_id = 'ropod_001'
        msg_type = "GET-ROBOTS-ASSIGNED-TO-TASK"
        message = self.send_request_get_response(msg_type, {'taskId': random_task_id})

        self.assertNotEqual(message, None)
        self.assertIn('header', message)
        self.assertIn('type', message['header'])
        self.assertEqual(message['header']['type'], msg_type)
        self.assertIn('payload', message)
        self.assertIn('robots', message['payload'])
        self.assertFalse(message['payload']['success'])

    def test_task_assigned_to_robot(self):
        robot_id = 'ropod_001'
        msg_type = "GET-TASKS-ASSIGNED-TO-ROBOT"
        message = self.send_request_get_response(msg_type, {'robotId': robot_id})

        self.assertNotEqual(message, None)
        self.assertIn('header', message)
        self.assertIn('type', message['header'])
        self.assertEqual(message['header']['type'], msg_type)
        self.assertIn('payload', message)
        self.assertIn('tasks', message['payload'])
        self.assertTrue(message['payload']['success'])
        self.assertGreater(len(message['payload']['tasks']), 0)

    def test_robot_status(self):
        robot_id = 'ropod_001'
        msg_type = "GET-ROBOT-STATUS"
        message = self.send_request_get_response(msg_type, {'robotId': robot_id})

        self.assertNotEqual(message, None)
        self.assertIn('header', message)
        self.assertIn('type', message['header'])
        self.assertEqual(message['header']['type'], msg_type)
        self.assertIn('payload', message)
        self.assertIn('status', message['payload'])
        self.assertTrue(message['payload']['success'])
        self.assertIn(robot_id, message['payload']['status'])

    def test_robot_assigned_to_task_positive(self):
        msg_type = "GET-ALL-SCHEDULED-TASK-IDS"
        message = self.send_request_get_response(msg_type)

        self.assertNotEqual(message, None)
        self.assertIn('header', message)
        self.assertIn('type', message['header'])
        self.assertEqual(message['header']['type'], msg_type)
        self.assertIn('payload', message)
        self.assertIn('taskIds', message['payload'])
        self.assertTrue(message['payload']['success'])

        if len(message['payload']['taskIds']) > 0:
            task_id = message['payload']['taskIds'][0]
            robot_id = 'ropod_001'
            msg_type = "GET-ROBOTS-ASSIGNED-TO-TASK"
            message = self.send_request_get_response(msg_type, {'taskId': task_id})

            self.assertNotEqual(message, None)
            self.assertIn('header', message)
            self.assertIn('type', message['header'])
            self.assertEqual(message['header']['type'], msg_type)
            self.assertIn('payload', message)
            self.assertIn('robots', message['payload'])
            self.assertTrue(message['payload']['success'])
            self.assertGreater(len(message['payload']['robots']), 0)

    def send_request_get_response(self, msg_type, payload_dict=None):
        self.test_pyre_node.send_request(msg_type, payload_dict)
        start_time = time.time()
        while self.test_pyre_node.response is None and \
                start_time + self.timeout_duration > time.time():
            time.sleep(0.2)
        message = self.test_pyre_node.response
        self.test_pyre_node.response = None
        return message

    def create_mock_db(self, filename, db_name):
        interfaces_dir = os.path.abspath(os.path.dirname(__file__))
        test_dir = os.path.dirname(os.path.dirname(interfaces_dir))
        db_file = os.path.join(test_dir, 'fixtures/db/' + filename)
        client = pm.MongoClient()

        # clear the database if it already exists
        if db_name in client.list_database_names():
            client.drop_database(db_name)

        db_obj = client[db_name]

        data=  None
        with open(db_file, "r") as file_obj:
            data = yaml.safe_load(file_obj)
        num_of_scheduled_tasks, num_of_ongoing_tasks, robots = 1, 1, 1
        try:
            num_of_scheduled_tasks = data['task_scheduled']
            num_of_ongoing_tasks = data['task_ongoing']
            robots = data['robots']
        except Exception as e:
            pass
        mongo_store = MongoStore(db_name)
        # print(num_of_scheduled_tasks, num_of_ongoing_tasks, robots)
        for i in robots:
            Robot.create_new(robot_id=i)
        for i in range(num_of_scheduled_tasks):
            task_uuid = uuid.uuid4()
            temp_index = i % len(robots)
            robots_assigned = [Robot.get_robot(robots[temp_index]), Robot.get_robot(robots[temp_index-1])]
            task = Task.create_new(task_id=task_uuid, assigned_robots=robots_assigned)
            task.update_status(TaskStatus.SCHEDULED)
        for i in range(num_of_ongoing_tasks):
            task_uuid = uuid.uuid4()
            temp_index = i % len(robots)
            robots_assigned = [Robot.get_robot(robots[temp_index]), Robot.get_robot(robots[temp_index-1])]
            task = Task.create_new(task_id=task_uuid, assigned_robots=robots_assigned)
            task.update_status(TaskStatus.ONGOING)

if __name__ == '__main__':
    unittest.main()
