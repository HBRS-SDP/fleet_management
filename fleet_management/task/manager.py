import datetime
import logging
import time

import inflection
from fleet_management.exceptions.osm import OSMPlannerException
from fleet_management.exceptions.planning import NoPlanFound
from fleet_management.resources.infrastructure.brsu import DurationGraph
from fmlib.models.requests import TransportationRequest
from fleet_management.db.models.task import TransportationTask as Task
from ropod.structs.status import TaskStatus
from fleet_management.db.models.robot import Ropod


class TaskManager(object):
    """An interface for handling ropod task requests and managing ropod tasks

    .. codeauthor:: Alex Mitrevski <aleksandar.mitrevski@h-brs.de>
    .. codeauthor:: Argentina Ortega <argentina.ortega@h-brs.de>

    """

    def __init__(self, ccu_store, api, **kwargs):
        self.ccu_store = ccu_store
        self.api = api
        self.logger = logging.getLogger("fms.task.manager")

        self.resource_manager = kwargs.get("resource_manager")
        self.dispatcher = kwargs.get("dispatcher")
        self.task_monitor = kwargs.get("task_monitor")
        self.duration_graph = kwargs.get("duration_graph")
        self.logger.info("Task Manager initialized...")

    def add_plugin(self, obj, name=None):
        if name:
            key = inflection.underscore(name)
        else:
            key = inflection.underscore(obj.__class__.__name__)
        self.__dict__[key] = obj
        self.logger.debug("Added %s plugin to %s", key, self.__class__.__name__)

    def configure(self, **kwargs):
        if self.resource_manager:
            self.logger.debug("Adding allocation interface")
            self._allocate = self.resource_manager.allocate

    def restore_task_data(self):
        """Loads any existing task data (ongoing tasks, scheduled tasks) from the CCU store database
        """
        pass

    def task_request_cb(self, msg):
        """
        Creates a task object from the received message
        and sends it to the processing function

        Args:
            msg (dict): A dictionary containing the message in a ROPOD format

        """
        payload = msg["payload"]

        self.logger.debug("Received task request %s ", payload.get("requestId"))
        try:
            task = self._process_request(payload)
        except (InvalidRequestLocation, InvalidRequestTime) as e:
            self.logger.error("Request %s is invalid" % payload.get("requestId"))
            return

        self.logger.debug("Processing task request")
        self._process_task(task)

    def _process_request(self, request):

        task_request = TransportationRequest.from_payload(request)
        try:
            self._validate_request(task_request)
        except InvalidRequestLocation as e:
            self.logger.error(*e.args)
            request_msg = {
                "header": {"type": "INVALID-TASK-REQUEST"},
                "payload": {"requestId": task_request.request_id},
            }
            self.api.publish(request_msg, groups=["ROPOD"])
            raise e

        task = Task.from_request(task_request)
        self.logger.debug(
            "Created task %s for request %s", task.task_id, task.request.request_id
        )
        return task

    def _validate_request(self, task_request):
        if task_request.pickup_location == task_request.delivery_location:
            raise InvalidRequestLocation("Pickup and delivery location are the same")
        elif task_request.latest_pickup_time < datetime.datetime.now():
            raise InvalidRequestTime(
                "Latest start time of %s is in the past"
                % task_request.latest_pickup_time
            )
        elif not self._is_valid_request_location(
            task_request.pickup_location, behaviour="docking"
        ):
            raise InvalidRequestLocation(
                "%s is not a valid pickup area." % task_request.pickup_location
            )
        elif not self._is_valid_request_location(
            task_request.delivery_location, behaviour="undocking"
        ):
            raise InvalidRequestLocation(
                "%s is not a valid delivery area." % task_request.delivery_location
            )

    def _is_valid_request_location(self, location, **kwargs):
        behaviour = kwargs.get("behaviour")
        try:
            self.path_planner.get_sub_area(location, behaviour=behaviour)
        except OSMPlannerException as e:
            return False
        return True

    def _process_task(self, task):
        """Processes a task before sending it to the robot(s).
        In the case of ROPOD, this method chooses robots for the task
        and generates an appropriate task plan

        Args:
            task (Task): A task object to be processed
        """

        try:
            task_plan = self._get_task_plan(task)
        except NoPlanFound as e:
            self.logger.error(e, exc_info=True)
            task.update_status(TaskStatus.PLANNING_FAILED)
            # TODO Communicate this back to the user
            return
        except OSMPlannerException:
            task.update_status(TaskStatus.PLANNING_FAILED)
            return  # TODO: this error needs to be communicated with the end user

        task.update_plan(task_plan)
        self.logger.debug("Task plan updated...")

        # TODO: Get estimated duration from planner
        if task_plan.mean is not None:
            mean = task_plan.mean
            variance = task_plan.variance
        else:
            mean, variance = self.get_task_duration_estimate(task_plan)

        self.logger.debug("Plan mean: %s, variance: %s", mean, variance)
        task.update_duration(mean=mean, variance=variance)

        self.logger.debug("Allocating robots for the task %s ", task.task_id)

        self._allocate(task)
        self.logger.debug("Sent to resource manager for allocation")

    def get_task_duration_estimate(self, task_plan):
        # TODO This is a hardcoded way to get the duration based on OSM and Guido runs
        mean, variance = self.duration_graph.get_duration(task_plan)
        return mean, variance

    def _allocate(self, *_, **__):
        self.logger.warning("No allocation interface configured")

    def _get_task_plan(self, task):
        self.logger.debug("Creating a task plan...")
        try:
            task_plan = self.task_planner.plan(task.request, self.path_planner)
            self.logger.debug("Planning successful for task %s", task.task_id)
        except OSMPlannerException:
            self.logger.error(
                "Path planning failed for task %s", task.task_id, exc_info=True
            )
            raise

        return task_plan

    def run(self):

        while len(self.resource_manager.allocations) > 0:
            task_id, robot_ids = self.resource_manager.allocations.pop()
            self.logger.debug("Reserving robots %s for task %s.", robot_ids, task_id)
            task = Task.get_task(task_id)

            ropods = [Ropod.get_robot(robot_id) for robot_id in robot_ids]
            task.assign_robots(ropods)

            # TODO: Get schedule from timetable.dispatchable_graph.
            # The schedule might change due to new allocations
            task_schedule = self.resource_manager.get_task_schedule(
                task_id, robot_ids[0]
            )
            task.update_schedule(task_schedule)

            self.logger.debug(
                "Task %s was allocated to %s. Start navigation time: %s Finish time: %s",
                task.task_id,
                [robot_id for robot_id in robot_ids],
                task.start_time,
                task.finish_time,
            )

        self.dispatcher.dispatch_tasks()


class TaskManagerError(Exception):
    pass


class InvalidRequest(TaskManagerError):
    pass


class InvalidRequestLocation(InvalidRequest):
    pass


class InvalidRequestTime(InvalidRequest):
    pass
