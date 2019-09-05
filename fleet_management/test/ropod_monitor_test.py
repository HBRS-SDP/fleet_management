import unittest

from ropod.structs.robot import Robot

from fleet_management.db.ccu_store import CCUStore
from fleet_management.resources.fleet.monitoring import FleetMonitor
from fleet_management.test.fixtures.utils import get_msg_from_path


class FleetMonitorTest(unittest.TestCase):
    def setUp(self):
        self.ccu_store = CCUStore('ropod_monitor_test')
        self.fleet_monitor = FleetMonitor(self.ccu_store, None)

        # Create ropod_001 from fixture
        robot_dict = get_msg_from_path('robot/robot.json')
        robot = Robot.from_dict(robot_dict)

        self.ccu_store.add_robot(robot)

    def test_register_robot(self):
        self.fleet_monitor.register_robot('ropod_002')
        self.assertIn('ropod_002', self.fleet_monitor.robots)

    def test_robot_status_cb(self):

        msg = get_msg_from_path('robot/status.json')
        payload = msg.get('payload')

        self.fleet_monitor.robot_status_cb(msg)
        robots = self.ccu_store.get_robots()
        robot = robots['ropod_001']
        robot_dict = robot.to_dict()

        self.maxDiff = None
        self.assertDictEqual(payload, robot_dict)

    def tearDown(self):
        self.ccu_store.clean()


if __name__ == '__main__':
    suite = unittest.TestLoader().loadTestsFromTestCase(FleetMonitorTest)
    unittest.TextTestRunner(verbosity=2).run(suite)
