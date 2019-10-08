import unittest

from fmlib.db.mongo import MongoStoreInterface, MongoStore
from fleet_management.db.models.robot import Ropod as Robot
from fleet_management.resources.fleet.monitoring import FleetMonitor
from fleet_management.test.fixtures.utils import get_msg_fixture


class FleetMonitorTest(unittest.TestCase):
    def setUp(self):
        store = MongoStore('fms_test', connectTimeoutMS=1)
        interface = MongoStoreInterface(store)
        self.fleet_monitor = FleetMonitor(interface, None)

        robot = Robot.create_new("ropod_001")
        self.fleet_monitor.robots['ropod_001'] = robot

    def test_register_robot(self):
        self.fleet_monitor.register_robot('ropod_002')
        self.assertIn('ropod_002', self.fleet_monitor.robots.keys())

    def test_robot_pose_cb(self):

        msg = get_msg_fixture('robot', 'robot-position.json')
        payload = msg.get('payload')
        pose = payload.get('pose')

        self.fleet_monitor.robot_pose_cb(msg)

        robot = self.fleet_monitor.robots.get(payload.get('robotId'))
        self.assertEqual(robot.position.area.name, payload.get('area'))
        self.assertEqual(robot.position.x, pose.get('x'))
        self.assertEqual(robot.position.y, pose.get('y'))
        self.assertEqual(robot.position.theta, pose.get('theta'))

    def tearDown(self):
        self.fleet_monitor.ccu_store.clean()


if __name__ == '__main__':
    suite = unittest.TestLoader().loadTestsFromTestCase(FleetMonitorTest)
    unittest.TextTestRunner(verbosity=2).run(suite)
