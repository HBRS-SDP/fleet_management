import argparse
import logging
import time

from fleet_management.config.loader import Configurator
from fleet_management.db.models.robot import Ropod
from fmlib.models.tasks import TransportationTask as Task
from ropod.structs.status import TaskStatus as TaskStatusConst


class RobotProxy(object):
    def __init__(self, robot_id, bidder, timetable_monitor, **kwargs):
        self.logger = logging.getLogger("fms.robot.proxy%s" % robot_id)

        self.robot_id = robot_id
        self.bidder = bidder
        self.timetable_monitor = timetable_monitor
        self.robot = Ropod.create_new(robot_id)

        self.api = kwargs.get("api")
        if self.api:
            self.api.register_callbacks(self)

        self.robot_store = kwargs.get("robot_store")

        self.logger.info("Initialized RobotProxy%s", robot_id)

    def configure(self, **kwargs):
        api = kwargs.get("api")
        robot_store = kwargs.get("robot_store")
        if api:
            self.api = api
            self.api.register_callbacks(self)
        if robot_store:
            self.robot_store = robot_store

    def robot_pose_cb(self, msg):
        payload = msg.get("payload")
        robot_id = payload.get("robotId")
        self.logger.debug("Robot proxy received robot pose")
        if robot_id == self.robot_id:
            self.robot.update_position(
                subarea=payload.get("subarea"), **payload.get("pose")
            )

    def task_cb(self, msg):
        payload = msg["payload"]
        assigned_robots = payload.get("assignedRobots")
        if self.robot_id in assigned_robots:
            task_id = payload.get("taskId")
            self.logger.debug("Received task %s", task_id)
            task = Task.get_task(task_id)
            task.update_status(TaskStatusConst.DISPATCHED)

    def run(self):
        try:
            self.api.start()
            while True:
                time.sleep(0.5)

        except (KeyboardInterrupt, SystemExit):
            self.logger.info("Terminating %s robot ...", self.bidder.robot_id)
            self.api.shutdown()
            self.logger.info("Exiting...")


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        type=str,
        default="osm",
        action="store",
        help="Path to the config file",
    )
    parser.add_argument("robot_id", type=str, help="example: ropod_001")
    args = parser.parse_args()

    config = args.config

    robot_id = args.robot_id

    config = Configurator(config)
    robot_components = config.configure_robot_proxy(robot_id)
    robot = RobotProxy(**robot_components)

    robot.run()
