from __future__ import print_function

import time
import unittest

from ropod.pyre_communicator.base_class import RopodPyre
from ropod.utils.models import RopodMessageFactory
from ropod.utils.uuid import generate_uuid

from fleet_management.config.loader import Configurator
from fleet_management.db.ccu_store import CCUStore


class QueryTest(RopodPyre):
    def __init__(self):
        super(QueryTest, self).__init__('resource_manager_test', ['ROPOD'], [], verbose=False, acknowledge=True)
        self.response = None
        self.start()

    def send_request(self, msg_type, payload_dict=None):
        query_msg = RopodMessageFactory.get_header(msg_type, recipients=[])

        query_msg['payload'] = {}
        query_msg['payload']['senderId'] = generate_uuid()
        if payload_dict is not None :
            for key in payload_dict.keys() :
                query_msg['payload'][key] = payload_dict[key]

        # print(json.dumps(query_msg, indent=2, default=str))
        self.shout(query_msg)

    def receive_msg_cb(self, msg_content):
        message = self.convert_zyre_msg_to_dict(msg_content)
        if message is None:
            return

        self.response = message


class ResourceManagerTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        config = Configurator()

        cls.ccu_store = CCUStore('sub_area_management_test')
        cls.osm_bridge = config.configure_osm_bridge()
        cls.resource_manager = config.configure_resource_manager(cls.ccu_store)
        cls.resource_manager.add_plugin('osm_bridge', cls.osm_bridge)
        cls.test_pyre_node = QueryTest()
        cls.timeout_duration = 3
        time.sleep(3)

    @classmethod
    def tearDownClass(cls):
        cls.resource_manager.shutdown()
        cls.ccu_store.delete_sub_areas()
        cls.test_pyre_node.shutdown()

    def setUp(self):
        pass

    def tearDown(self):
        self.ccu_store.delete_sub_area_reservations()

    def send_request_get_response(self, msg_type, payload_dict = None):
        self.test_pyre_node.send_request(msg_type, payload_dict)
        start_time = time.time()
        while self.test_pyre_node.response is None and \
                start_time + self.timeout_duration > time.time():
            time.sleep(0.2)
        message = self.test_pyre_node.response
        self.test_pyre_node.response = None
        return message

    #def test_some_behaviour(self):
    #    msg_type = "SOME-MSG-TYPE"
    #    message = self.send_request_get_response(msg_type)

    #    self.assertNotEqual(message, None)
    #    self.assertIn('header', message)
    #    self.assertIn('type', message['header'])
    #    self.assertEqual(message['header']['type'], msg_type)
    #    self.assertIn('payload', message)
    #    #TODO: a bunch of more assert statements


if __name__ == '__main__':
    unittest.main()
