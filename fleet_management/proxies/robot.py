import argparse
import logging
import time

from fleet_management.config.builder import robot_builder
from fmlib.config.params import ConfigParams as ConfigParamsBase


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
            self.logger.info("Terminating %s robot ...", self.bidder.id)
            self.api.shutdown()
            self.logger.info("Exiting...")


class ConfigParams(ConfigParamsBase):
    default_config_module = 'fleet_management.config.default'


default_config = ConfigParams.default()


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--file', type=str, action='store', help='Path to the config file')
    parser.add_argument('robot_id', type=str, help='example: ropod_001')
    args = parser.parse_args()

    config_file_path = args.file
    robot_id = args.robot_id

    if config_file_path is None:
        config_params = default_config
    else:
        config_params = ConfigParams.from_file(config_file_path)

    logger_config = config_params.get('logger')
    logging.config.dictConfig(logger_config)

    robot_components = robot_builder(robot_id, config_params)
    robot = RobotProxy(robot_id, **robot_components)
    robot.run()

