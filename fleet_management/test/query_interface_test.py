from __future__ import print_function

import time
import unittest

from ropod.pyre_communicator.base_class import RopodPyre
from ropod.utils.models import RopodMessageFactory
from ropod.utils.uuid import generate_uuid

from fleet_management.config.loader import Config
from fleet_management.db.query_interface import FleetManagementQueryInterface


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
        config = Config(config_file=None, initialize=False)

        zyre_config = {'node_name': 'ccu_query_interface',
                       'groups': ['ROPOD'],
                       'message_types': list()}
        db_name = config.config_params.get('ccu_store').get('db_name')
        cls.query_interface = FleetManagementQueryInterface(zyre_config, db_name)

        cls.test_pyre_node = QueryTest()
        cls.timeout_duration = 3
        time.sleep(3)

    @classmethod
    def tearDownClass(cls):
        cls.query_interface.shutdown()
        cls.test_pyre_node.shutdown()

    def setUp(self):
        pass

    def tearDown(self):
        pass

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

    def send_request_get_response(self, msg_type, payload_dict=None):
        self.test_pyre_node.send_request(msg_type, payload_dict)
        start_time = time.time()
        while self.test_pyre_node.response is None and \
                start_time + self.timeout_duration > time.time():
            time.sleep(0.2)
        message = self.test_pyre_node.response
        self.test_pyre_node.response = None
        return message


if __name__ == '__main__':
    unittest.main()
