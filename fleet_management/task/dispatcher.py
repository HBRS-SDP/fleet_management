import logging

import inflection
from fleet_management.db.models.actions import GoTo
from fleet_management.db.models.environment import Area
from fleet_management.db.models.task import TransportationTask as Task
from fleet_management.exceptions.osm import OSMPlannerException
from ropod.structs.status import TaskStatus as TaskStatusConst


class Dispatcher:
    def __init__(self, ccu_store, api, **kwargs):
        self.logger = logging.getLogger('fms.task.dispatcher')
        self.ccu_store = ccu_store
        self.api = api
        self.scheduled_tasks = dict()

    def add_plugin(self, obj, name=None):
        if name:
            key = inflection.underscore(name)
        else:
            key = inflection.underscore(obj.__class__.__name__)
        self.__dict__[key] = obj
        self.logger.debug("Added %s plugin to %s", key, self.__class__.__name__)

    def dispatch_tasks(self):
        """
        Dispatches all allocated tasks that are ready for dispatching
        """
        allocated_tasks = Task.get_tasks_by_status(TaskStatusConst.ALLOCATED)
        for task in allocated_tasks:
            if task.is_executable():
                self.logger.info('Dispatching task %s', task.task_id)
                for robot in task.assigned_robots:
                    self._add_pre_task_action(robot, task)
                    if task.status == TaskStatusConst.PLANNING_FAILED:
                        # TODO: Remove task. Notify user and ask whether to re-allocate the task or not, and
                        # with which constraints (new constraints defined by the user or asap)
                        pass
                    else:
                        self.dispatch_task(task, robot.robot_id)
                task.update_status(TaskStatusConst.DISPATCHED)

    def _add_pre_task_action(self, robot, task):
        self.logger.debug("Adding pre task action to plan for task %s robot %s", task.task_id, robot.robot_id)
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
            pickup_subarea = self.path_planner.get_sub_area(task.request.pickup_location, behaviour="docking")
            self.logger.debug('Planning path between %s and %s', robot.position.subarea.name, pickup_subarea.name)

            areas = self.path_planner.get_path_plan_from_local_area(robot.position.subarea.name, pickup_subarea.name)
            path_plan = list()

            for area in areas:
                model_area = Area(**area.to_dict())
                path_plan.append(model_area)

        except Exception as e:
            self.logger.error("Path planner error", exc_info=True)
            raise OSMPlannerException("Task planning failed") from e
        return path_plan

    def dispatch_task(self, task, robot_id):
        """
        Sends a task to the appropriate robot fleet

        Args:
            task: a ropod.structs.task.Task object
            robot_id: a robot UUID
        """
        self.logger.info("Dispatching task to robot %s", robot_id)
        task_msg = self.api.create_message(task)

        task_msg["payload"].pop("constraints")
        task_msg["payload"]["assignedRobots"] = [robot.robot_id for robot in task.assigned_robots]
        task_msg["payload"]["plan"][0]["_id"] = task_msg["payload"]["plan"][0].pop("robot")

        self.api.publish(task_msg, groups=['ROPOD'])

