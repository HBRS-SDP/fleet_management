import time
import unittest

from fleet_management.db.mongo import MongoStore, MongoStoreInterface
from fleet_management.config.loader import Configurator, default_config
from fleet_management.resources.infrastructure.elevators.monitor import ElevatorMonitor


class TestElevatorMonitor(unittest.TestCase):

    def setUp(self):
        config = Configurator()
        store = MongoStore('fms_test', connectTimeoutMS=1)
        self.ccu_store = MongoStoreInterface(store)
        self.api = config.api

    def test_receive_elevator_status_msg(self):
        self.monitor = ElevatorMonitor('1', self.ccu_store, self.api)
        api_config = default_config.component('elevator_manager').get('monitors')
        self.monitor.configure_api(**api_config)

        self.monitor.api.start()
        time.sleep(3)
        elevator = self.monitor.elevator
        self.assertEqual(elevator._model.status.is_available, True)
        self.assertEqual(elevator._model.status.door_open_at_start_floor, False)
        self.assertEqual(elevator._model.status.door_open_at_goal_floor, False)
        self.assertEqual(elevator._model.status.calls, 0)
        # TODO the simulated elevator starts randomly with a new floor, ignoring this for now
        # self.assertEqual(elevator._model.status.floor, 8)

    def tearDown(self):
        self.monitor.api.shutdown()
        self.ccu_store.clean()
