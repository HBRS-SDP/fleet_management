import argparse
import logging
import time

from fleet_management.config.loader import Configurator


class RobotProxy(object):
    def __init__(self, robot_id, bidder, **kwargs):
        self.logger = logging.getLogger('fms.robot.proxy%s' % robot_id)

        self.robot_id = robot_id
        self.bidder = bidder

        self.api = kwargs.get('api')
        if self.api:
            self.api.register_callbacks(self)

        self.robot_store = kwargs.get('robot_store')

        self.logger.info("Initialized RobotProxy%s", robot_id)

    def configure(self, **kwargs):
        api = kwargs.get('api')
        robot_store = kwargs.get('robot_store')
        if api:
            self.api = api
            self.api.register_callbacks(self)
        if robot_store:
            self.robot_store = robot_store

    def run(self):
        try:
            self.api.start()
            while True:
                time.sleep(0.5)

        except (KeyboardInterrupt, SystemExit):
            self.logger.info("Terminating %s robot ...", self.bidder.robot_id)
            self.api.shutdown()
            self.logger.info("Exiting...")


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--file', type=str, action='store', help='Path to the config file')
    parser.add_argument('robot_id', type=str, help='example: ropod_001')
    args = parser.parse_args()

    config_file_path = args.file
    robot_id = args.robot_id

    config = Configurator(config_file_path)
    robot_components = config.configure_robot_proxy(robot_id)
    robot = RobotProxy(robot_id, **robot_components)
    robot.run()

