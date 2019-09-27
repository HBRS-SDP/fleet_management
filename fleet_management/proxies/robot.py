import argparse
import logging
import time

from fleet_management.config.builder import robot_builder
from fleet_management.config.loader import Configurator


class RobotProxy(object):
    def __init__(self, robot_id, api, robot_store, bidder, **kwargs):
        self.logger = logging.getLogger('fms.robot.proxy%s' % robot_id)

        self.robot_id = robot_id
        self.api = api
        self.robot_store = robot_store
        self.bidder = bidder
        self.api.register_callbacks(self)

        self.logger.info("Initialized RobotProxy%s", robot_id)

    def run(self):
        try:
            self.api.start()
            while True:
                time.sleep(0.5)

        except (KeyboardInterrupt, SystemExit):
            self.logger.info("Terminating %s robot ...", self.bidder.id)
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

    robot_components = robot_builder.configure(robot_id, config._config_params)
    robot = RobotProxy(robot_id, **robot_components)
    robot.run()

