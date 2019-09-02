import argparse
import logging
import time

import rospy

from fleet_management.config.loader import Configurator


class FMS(object):
    def __init__(self, config_file=None):
        self.logger = logging.getLogger('fms')

        self.logger.info("Configuring FMS ...")
        self.config = Configurator(config_file)
        self.config.configure()
        self.config.configure_logger()
        self.ccu_store = self.config.ccu_store
        self.task_manager = self.config.task_manager
        self.resource_manager = self.config.resource_manager

        self.api = self.config.api
        self.api.register_callbacks(self)

        self.task_manager.restore_task_data()
        self.logger.info("Initialized FMS")

    def run(self):
        try:
            self.api.start()

            while True:
                self.task_manager.dispatcher.dispatch_tasks()
                self.task_manager.process_task_requests()
                self.resource_manager.run()
                self.api.run()
                time.sleep(0.5)
        except (KeyboardInterrupt, SystemExit):
            rospy.signal_shutdown('FMS ROS shutting down')
            self.api.shutdown()
            self.logger.info('FMS is shutting down')

    def shutdown(self):
        self.api.shutdown()


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--file', type=str, action='store', help='Path to the config file')
    args = parser.parse_args()
    config_file_path = args.file

    fms = FMS(config_file_path)

    fms.run()
