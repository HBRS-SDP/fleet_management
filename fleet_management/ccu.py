import argparse
import time
import logging

import rospy

from fleet_management.config.loader import Config


class FMS(object):
    def __init__(self, config_file=None):
        self.logger = logging.getLogger('fms')

        self.logger.info("Configuring FMS ...")
        self.config = Config(config_file, initialize=True)
        self.config.configure_logger()
        self.ccu_store = self.config.ccu_store

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
        # TODO Add this to config file and read it at start up
        self.api.zyre.add_callback(self, 'TASK-REQUEST', 'task_manager', 'task_request_cb')
        self.api.zyre.add_callback(self, 'TASK-PROGRESS', 'task_manager', 'task_progress_cb')
        self.api.zyre.add_callback(self, 'ROBOT-ELEVATOR-CALL-REQUEST', 'resource_manager', 'elevator_call_request_cb')
        self.api.zyre.add_callback(self, 'ELEVATOR-CMD-REPLY', 'resource_manager', 'elevator_cmd_reply_cb')
        self.api.zyre.add_callback(self, 'ROBOT-CALL-UPDATE', 'resource_manager', 'robot_call_update_cb')
        self.api.zyre.add_callback(self, 'ELEVATOR-STATUS', 'resource_manager', 'elevator_status_cb')
        self.api.zyre.add_callback(self, 'ROBOT-UPDATE', 'resource_manager', 'robot_update_cb')
        self.api.zyre.add_callback(self, 'SUB-AREA-RESERVATION', 'resource_manager', 'subarea_reservation_cb')
        self.api.zyre.add_callback(self, 'BID', 'auctioneer', 'bid_cb')
        self.api.zyre.add_callback(self, 'NO-BID', 'auctioneer', 'no_bid_cb')
        self.api.zyre.add_callback(self, 'SCHEDULE', 'auctioneer', 'schedule_cb')
        self.api.zyre.add_callback(self, 'TASK-ALTERNATIVE-TIMESLOT', 'auctioneer', 'alternative_timeslot_cb')

        self.task_manager.restore_task_data()
        self.logger.info("Initialized FMS")

    def run(self):
        try:
            if self.api.zyre:
                self.api.zyre.start()
            if self.api.ros:
                self.api.ros.start()

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
            self.logger.info('FMS is shutting down')

    def shutdown(self):
        self.api.zyre.shutdown()


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--file', type=str, action='store', help='Path to the config file')
    args = parser.parse_args()
    config_file_path = args.file

    fms = FMS(config_file_path)

    fms.run()
