import unittest

from fleet_management.db.ccu_store import CCUStore
from fleet_management.resources.fleet.monitoring import FleetMonitor
from fleet_management.test.fixtures.utils import get_msg_from_path


class FleetMonitorTest(unittest.TestCase):
    def setUp(self):
        self.ccu_store = CCUStore('ropod_ccu_store')
        self.fleet_monitor = FleetMonitor(self.ccu_store, None)

    def test_register_robot(self):
        self.fleet_monitor.register_robot('ropod_001')
        self.assertIn('ropod_001', self.fleet_monitor.robots)

    def test_robot_status_cb(self):
        msg = get_msg_from_path('robot/status.json')
        self.fleet_monitor.robot_status_cb(msg)
        robots = self.ccu_store.get_robots()
        robot = robots['ropod_001']
        status = robot.status.to_dict()

        payload = msg.get('payload')

        self.assertDictEqual(payload, status)


if __name__ == '__main__':
    suite = unittest.TestLoader().loadTestsFromTestCase(FleetMonitorTest)
    unittest.TextTestRunner(verbosity=2).run(suite)

