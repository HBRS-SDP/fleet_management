import argparse
import time
import logging
import rospy

from fleet_management.config.loader import Config, register_api_callbacks


class FMS(object):
    def __init__(self, config_file=None):
        self.logger = logging.getLogger('fms')

        self.logger.info("Configuring FMS ...")
        self.config = Config(config_file, initialize=True)
        self.config.configure_logger()
        self.ccu_store = self.config.ccu_store
        self.threads = list()

        plugins = self.config.configure_plugins(self.ccu_store)
        for plugin_name, plugin in plugins.items():
            self.__dict__[plugin_name] = plugin

        self.task_manager = self.config.configure_task_manager(self.ccu_store)
        self.task_manager.add_plugin('osm_bridge', plugins.get('osm_bridge'))
        self.task_manager.add_plugin('path_planner', plugins.get('path_planner'))
        self.task_manager.add_plugin('task_planner', plugins.get('task_planner'))

        fleet = self.config.config_params.get('resources').get('fleet')

        self.resource_manager = self.config.configure_resource_manager(self.ccu_store)
        self.resource_manager.add_plugin('osm_bridge', plugins.get('osm_bridge'))
        self.resource_manager.add_plugin('auctioneer', plugins.get('auctioneer'))

        self.task_manager.add_plugin('resource_manager', self.resource_manager)

        self.api = self.config.api

        register_api_callbacks(self, self.api)

        self.task_manager.restore_task_data()
        self.logger.info("Initialized FMS")

    def run(self):
        try:
            self.api.start()

            while True:
                self.resource_manager.auctioneer.run()
                self.resource_manager.get_allocation()
                self.task_manager.process_task_requests()
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
