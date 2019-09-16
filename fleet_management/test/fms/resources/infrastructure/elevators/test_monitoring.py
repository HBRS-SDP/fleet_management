import unittest

from fleet_management.db.mongo import MongoStore, MongoStoreInterface
from fleet_management.resources.infrastructure.elevators.monitor import ElevatorMonitor
from fleet_management.test.fixtures.utils import get_msg_fixture


class TestElevatorMonitor(unittest.TestCase):
    def setUp(self):
        store = MongoStore('fms_test', connectTimeoutMS=1)
        interface = MongoStoreInterface(store)
        self.elevator_monitor = ElevatorMonitor(1, interface, None)

    def test_elevator_status_cb(self):
        msg = get_msg_fixture('elevator', 'ropod-elevator-status-A.json')
        payload = msg.get('payload')
        self.elevator_monitor.elevator_status_cb(msg)
        self.assertEqual(self.elevator_monitor.elevator._model.status.calls, 1)
        self.assertEqual(self.elevator_monitor.elevator._model.status.floor, 10)
        self.assertEqual(self.elevator_monitor.elevator._model.status.is_available, False)
        self.assertEqual(self.elevator_monitor.elevator._model.status.door_open_at_goal_floor, False)
        self.assertEqual(self.elevator_monitor.elevator._model.status.door_open_at_start_floor, True)

        msg = get_msg_fixture('elevator', 'ropod-elevator-status-B.json')
        payload = msg.get('payload')
        self.elevator_monitor.elevator_status_cb(msg)
        self.assertEqual(self.elevator_monitor.elevator._model.status.calls, 1)
        self.assertEqual(self.elevator_monitor.elevator._model.status.floor, 8)
        self.assertEqual(self.elevator_monitor.elevator._model.status.is_available, True)
        self.assertEqual(self.elevator_monitor.elevator._model.status.door_open_at_goal_floor, False)
        self.assertEqual(self.elevator_monitor.elevator._model.status.door_open_at_start_floor, False)

    def tearDown(self):
        self.elevator_monitor.ccu_store.clean()
