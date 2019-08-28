import time
import unittest

from fleet_management.config.loader import Configurator, default_config
from fleet_management.resources.infrastructure.elevators.monitor import ElevatorMonitor


class TestElevatorMonitor(unittest.TestCase):

    def setUp(self):
        config = Configurator()
        api_config = default_config.component('elevator_manager').get('monitors')
        self.ccu_store = config.ccu_store
        self.monitor = ElevatorMonitor(1, self.ccu_store, config.api)
        self.monitor.configure_api(**api_config)

        self.monitor.api.start()

    def test_receive_elevator_status_msg(self):
        time.sleep(3)
        elevator = self.ccu_store.get_elevators().get(1)
        print(elevator.is_available)
        self.assertEqual(elevator.is_available, True)
        self.assertEqual(elevator.door_open_at_start_floor, False)
        self.assertEqual(elevator.door_open_at_goal_floor, False)
        self.assertEqual(elevator.elevator_id, 1)
        self.assertEqual(elevator.calls, 0)
        self.assertEqual(elevator.floor, 9)

    def tearDown(self):
        self.monitor.api.shutdown()
        self.ccu_store.clean()
