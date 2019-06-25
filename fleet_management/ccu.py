import time
import os.path
import logging

from ropod.utils.logging.config import config_logger

from fleet_management.config.loader import Config


class FMS(object):
    def __init__(self, config_file):
        self.logger = logging.getLogger('fms')

        self.logger.info("Configuring FMS ...")
        self.config = Config(config_file)
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

        self.zyre_api = self.config.api
        # TODO Add this to config file and read it at start up
        self.zyre_api.add_callback(self, 'TASK-REQUEST', 'task_manager', 'task_request_cb')
        self.zyre_api.add_callback(self, 'TASK-PROGRESS', 'task_manager', 'task_progress_cb')
        self.zyre_api.add_callback(self, 'ROBOT-ELEVATOR-CALL-REQUEST', 'resource_manager', 'elevator_call_request_cb')
        self.zyre_api.add_callback(self, 'ELEVATOR-CMD-REPLY', 'resource_manager', 'elevator_cmd_reply_cb')
        self.zyre_api.add_callback(self, 'ROBOT-CALL-UPDATE', 'resource_manager', 'robot_call_update_cb')
        self.zyre_api.add_callback(self, 'ELEVATOR-STATUS', 'resource_manager', 'elevator_status_cb')
        self.zyre_api.add_callback(self, 'ROBOT-UPDATE', 'resource_manager', 'robot_update_cb')
        self.zyre_api.add_callback(self, 'SUB-AREA-RESERVATION', 'resource_manager', 'subarea_reservation_cb')
        self.zyre_api.add_callback(self, 'BID', 'auctioneer', 'bid_cb')
        self.zyre_api.add_callback(self, 'NO-BID', 'auctioneer', 'no_bid_cb')
        self.zyre_api.add_callback(self, 'SCHEDULE', 'auctioneer', 'schedule_cb')
        self.zyre_api.add_callback(self, 'TASK-ALTERNATIVE-TIMESLOT', 'auctioneer', 'alternative_timeslot_cb')

        self.task_manager.restore_task_data()
        self.logger.info("Initialized FMS")

    def run(self):
        try:
            self.task_manager.start()
            while True:
                self.task_manager.dispatch_tasks()
                # self.task_manager.resend_message_cb()
                if self.zyre_api.acknowledge:
                    self.zyre_api.resend_message_cb()
                time.sleep(0.5)
        except (KeyboardInterrupt, SystemExit):
            self.task_manager.shutdown()
            self.zyre_api.shutdown()
            self.logger.info('FMS is shutting down')


if __name__ == '__main__':
    code_dir = os.path.abspath(os.path.dirname(__file__))
    main_dir = os.path.dirname(code_dir)

    log_config_file = os.path.join(main_dir, 'config/logging.yaml')
    config_logger(log_config_file)

    config_file = os.path.join(main_dir, "config/fms_config-v2.yaml")

    fms = FMS(config_file)

    fms.run()

