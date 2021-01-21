import copy
import logging
from datetime import timedelta

import inflection
from fleet_management.db.models.actions import GoTo
from fleet_management.db.models.environment import Area
from fleet_management.db.models.robot import Ropod
from fleet_management.exceptions.osm import OSMPlannerException
from ropod.structs.status import TaskStatus as TaskStatusConst
from ropod.utils.timestamp import TimeStamp


class Dispatcher:
    def __init__(self, ccu_store, api, **kwargs):
        self.logger = logging.getLogger("fms.task.dispatcher")
        self.ccu_store = ccu_store
        self.api = api
        self.freeze_window = timedelta(minutes=kwargs.get("freeze_window", 0.5))
        self.n_queued_tasks = kwargs.get("n_queued_tasks", 3)
        self.d_graph_updates = dict()

    def add_plugin(self, obj, name=None):
        if name:
            key = inflection.underscore(name)
        else:
            key = inflection.underscore(obj.__class__.__name__)
        self.__dict__[key] = obj
        self.logger.debug("Added %s plugin to %s", key, self.__class__.__name__)

    def is_schedulable(self, start_time):
        current_time = TimeStamp()
        if start_time.get_difference(current_time) < self.freeze_window:
            return True
        return False

    def dispatch_tasks(self):
        """
        Dispatches earliest task in each robot's timetable that is ready for dispatching
        """
        for robot_id, timetable in self.timetable_manager.items():
            task = timetable.get_earliest_task()
            if task and task.status.status == TaskStatusConst.ALLOCATED:
                start_time = timetable.get_start_time(task.task_id)
                if self.is_schedulable(start_time):
                    robot = Ropod.get_robot(robot_id)
                    self._add_pre_task_action(robot, task)

                    if task.status.status == TaskStatusConst.PLANNING_FAILED:
                        # TODO: Remove task. Notify user and ask whether to re-allocate the task or not, and
                        # with which constraints (new constraints defined by the user or asap)
                        pass
                    else:
                        self.send_d_graph_update(timetable)
                        self.dispatch_task(task, robot_id)

    def _add_pre_task_action(self, robot, task):
        self.logger.debug(
            "Adding pre task action to plan for task %s robot %s",
            task.task_id,
            robot.robot_id,
        )
        try:
            path_plan = self._get_pre_task_path_plan(robot, task)
        except OSMPlannerException:
            task.update_status(TaskStatusConst.PLANNING_FAILED)
            return

        pre_task_action = GoTo.create_new(type="GOTO", areas=path_plan)

        task.plan[0].actions.insert(0, pre_task_action)
        task.save()

    def _get_pre_task_path_plan(self, robot, task):
        try:
            pickup_subarea = self.path_planner.get_sub_area(
                task.request.pickup_location, behaviour="docking"
            )

        except Exception as e:
            self.logger.error("Path planner error", exc_info=True)
            raise OSMPlannerException("Task planning failed") from e

        try:
            self.logger.debug(
                "Planning path between %s and %s",
                robot.position.subarea.name,
                pickup_subarea.name,
            )

            path_plan = self.path_planner.get_path_plan_from_local_area(
                robot.position.subarea.name, pickup_subarea.name
            )
            # path_plan = list()
            # try:
            #     for area in areas[0]:
            #         model_area = Area(**area.to_dict())
            #         path_plan.append(model_area)
            # except TypeError:
            #     for area in areas:
            #         model_area = Area(**area.to_dict())
            #         path_plan.append(model_area)

        except Exception as e:
            self.logger.error("Path planner error", exc_info=True)
            raise OSMPlannerException("Task planning failed") from e
        return path_plan.areas

    def send_d_graph_update(self, timetable):
        prev_d_graph_update = self.d_graph_updates.get(timetable.robot_id)
        d_graph_update = timetable.get_d_graph_update(
            timetable.robot_id, self.n_queued_tasks
        )

        if prev_d_graph_update != d_graph_update:
            self.logger.debug("Sending DGraphUpdate to %s", timetable.robot_id)
            msg = self.api.create_message(d_graph_update)
            self.api.publish(msg, peer=timetable.robot_id + "_")
            self.d_graph_updates[timetable.robot_id] = copy.deepcopy(d_graph_update)

    def dispatch_task(self, task, robot_id):
        """
        Sends a task to the appropriate robot fleet

        Args:
            task: a ropod.structs.task.Task object
            robot_id: a robot UUID
        """
        self.logger.info("Dispatching task to robot %s", robot_id)
        task.update_status(TaskStatusConst.DISPATCHED)
        task_msg = self.api.create_message(task)

        task_msg["payload"].pop("request")
        task_msg["payload"]["assignedRobots"] = [
            robot.robot_id for robot in task.assigned_robots
        ]

        # Dispatch task to schedule_execution_monitor
        # TODO: Combine task and dgraph_update and send it to the com_mediator
        self.api.publish(task_msg, peer=robot_id + "_")
