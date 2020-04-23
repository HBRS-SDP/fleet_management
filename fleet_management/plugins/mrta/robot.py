import argparse
import time
import logging
from fleet_management.config.loader import Configurator
from fmlib.models.tasks import TransportationTask as Task


class Robot:
    def __init__(self, robot_id, api, schedule_execution_monitor, **kwargs):

        self.robot_id = robot_id
        self.api = api
        self.schedule_execution_monitor = schedule_execution_monitor

        self.api.register_callbacks(self)
        self.logger = logging.getLogger('mrs.robot.%s' % robot_id)
        self.logger.info("Initialized Robot %s", robot_id)

    def run(self):
        try:
            self.api.start()
            while True:
                tasks = Task.get_tasks_by_robot(self.robot_id)
                if self.schedule_execution_monitor.task is None:
                    self.schedule_execution_monitor.process_tasks(tasks)
                time.sleep(0.5)
        except (KeyboardInterrupt, SystemExit):
            self.logger.info("Terminating %s robot ...", self.robot_id)
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
    robot_components = config.configure_robot(robot_id)
    robot = Robot(**robot_components)

    robot.run()

