import argparse
import time
import logging
import threading

import rospy

from fleet_management.config.loader import Config


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
        self.register_api_callbacks(self.api)

        self.task_manager.restore_task_data()
        self.logger.info("Initialized FMS")

    def run(self):
        try:
            if self.api.zyre:
                self.api.zyre.start()
            if self.api.ros:
                self.api.ros.start()
            if self.api.rest:
                x = threading.Thread(target=self.api.rest.start)
                self.threads.append(x)
                x.start()

            while True:
                if self.api.ros:
                    if not rospy.is_shutdown():
                        self.api.ros.run()
                self.task_manager.dispatch_tasks()
                self.resource_manager.auctioneer.run()
                self.resource_manager.get_allocation()
                self.task_manager.process_task_requests()
                if self.api.zyre:
                    if self.api.zyre.acknowledge:
                        self.api.zyre.resend_message_cb()
                time.sleep(0.5)
        except (KeyboardInterrupt, SystemExit):
            rospy.signal_shutdown('FMS ROS shutting down')
            self.api.zyre.shutdown()
            self.api.rest.shutdown()
            self.threads[0].join()
            self.logger.info('FMS is shutting down')

    def shutdown(self):
        self.api.zyre.shutdown()

    def register_api_callbacks(self, api):
        for option in api.middleware_collection:
            callbacks = api.config_params.get(option).get('callbacks', list())
            for callback in callbacks:
                component = callback.pop('component', None)
                function = self.__get_callback_function(component)
                api.register_callback(option, function, **callback)

    def __get_callback_function(self, component):
        objects = component.split('.')
        child = objects.pop(0)
        parent = getattr(self, child)
        while objects:
            child = objects.pop(0)
            parent = getattr(parent, child)

        return parent


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--file', type=str, action='store', help='Path to the config file')
    args = parser.parse_args()
    config_file_path = args.file

    fms = FMS(config_file_path)

    fms.run()
