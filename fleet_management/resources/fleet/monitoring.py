import logging

from fmlib.db.mongo import MongoStore, MongoStoreInterface
from fleet_management.db.models.robot import Ropod as Robot


class FleetMonitor:

    def __init__(self, ccu_store, api, **kwargs):
        self.logger = logging.getLogger('fms.resources.fleet.monitoring')
        self.ccu_store = ccu_store
        self.api = api
        self.robots = dict()

        robot_config = kwargs.get('robots', None)
        if robot_config:
            for robot_type in robot_config:
                print(robot_type)

        api_config = kwargs.get('api', None)
        if api_config:
            self.__configure_api(api_config)

        self.logger.debug("Initialized Fleet Monitor")

    def register_robot(self, robot_id):
        """Adds the robot to the list of robots it will track.
        This method also initializes all the required documents in MongoDB

        Args:
            robot_id: The ID of the robot to register

        """
        robot = Robot.create_new(robot_id)
        self.robots[robot_id] = robot

    def robot_pose_cb(self, msg):
        payload = msg.get('payload')
        robot_id = payload.get('robotId')
        robot = self.robots.get(robot_id)
        robot.update_position(subarea=payload.get('subarea'), **payload.get('pose'))

    def __configure_api(self, api_config):
        self.api.register_callbacks(self, api_config)


if __name__ == '__main__':
    store = MongoStore('fms_test', connectTimeoutMS=1)
    interface = MongoStoreInterface(store)
    monitor = FleetMonitor(interface, None)
    monitor.register_robot("ropod_001")
