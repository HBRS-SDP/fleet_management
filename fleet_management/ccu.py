import time
import os.path
import logging
from ropod.utils.logging.config import config_logger

from fleet_management.config.loader import Config
from fleet_management.task_manager import TaskManager


class FMS(object):
    def __init__(self, config_file):
        self.logger = logging.getLogger('fms')

        self.logger.info("Configuring FMS ...")
        self.config = Config(config_file)
        self.ccu_store = self.config.configure_ccu_store()

        self.logger.info("Initialized FMS")

    def run(self):
        try:
            while True:
                time.sleep(0.5)
        except (KeyboardInterrupt, SystemExit):
            self.logger.info('FMS is shutting down')


if __name__ == '__main__':
    code_dir = os.path.abspath(os.path.dirname(__file__))
    main_dir = os.path.dirname(code_dir)

    log_config_file = os.path.join(main_dir, 'config/logging.yaml')
    config_logger(log_config_file)

    config_file = os.path.join(main_dir, "config/fms_config-v2.yaml")

    fms = FMS(config_file)

    fms.run()

