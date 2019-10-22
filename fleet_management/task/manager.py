import logging

import inflection
from fleet_management.exceptions.osm import OSMPlannerException
from fmlib.models.requests import TransportationRequest
from fmlib.models.tasks import Task


class TaskManager(object):
    """An interface for handling ropod task requests and managing ropod tasks

    .. codeauthor:: Alex Mitrevski <aleksandar.mitrevski@h-brs.de>
    .. codeauthor:: Argentina Ortega <argentina.ortega@h-brs.de>

    """

    def __init__(self, ccu_store, api, **kwargs):
        self.ccu_store = ccu_store
        self.api = api
        self.logger = logging.getLogger("fms.task.manager")

        self.unallocated_tasks = dict()
        self.resource_manager = kwargs.get('resource_manager')
        self.dispatcher = kwargs.get('dispatcher')
        self.task_monitor = kwargs.get('task_monitor')
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
        payload = msg['payload']

        self.logger.debug('Received task request %s ', payload.get('requestId'))
        task_request = TransportationRequest.from_payload(payload)
        task = Task.from_request(task_request)
        self.logger.debug('Created task %s for request %s', task.task_id,
                          task.request.request_id)

        self.logger.debug("Processing task request")
        self._process_task(task)

    def _process_task(self, task):
        """Processes a task before sending it to the robot(s).
        In the case of ROPOD, this method chooses robots for the task
        and generates an appropriate task plan

        Args:
            task (Task): A task object to be processed
        """

        task_plan = self._get_task_plan(task)


        # Assuming a constant velocity of 1m/s, the estimated duration of the task is the estimated distance

        self.logger.debug('Allocating robots for the task %s ', task.task_id)
        self.unallocated_tasks[task.task_id] = {'task': task,
                                                'plan': task_plan
                                                }

        self._allocate(task)
        self.logger.debug('Sent to resource manager for allocation')

    def _allocate(self, *_, **__):
        self.logger.warning("No allocation interface configured")

    def _get_task_plan(self, task):
        self.logger.debug('Creating a task plan...')
        try:
            task_plan = self.task_planner.plan(task.request, self.path_planner)
            self.logger.debug('Planning successful for task %s', task.task_id)
        except OSMPlannerException as e:
            self.logger.error(str(e))
            self.logger.error("There is an error with the OSM planner. "
                              "Can't process task request")
            return  # TODO: this error needs to be communicated with the end user

        return task_plan

    def run(self):

        while self.resource_manager.allocations:
            task_id, robot_ids = self.resource_manager.allocations.pop()
            self.logger.debug('Reserving robots %s for task %s.', robot_ids, task_id)
            request = self.unallocated_tasks.pop(task_id)

            task = request.get('task')
            task_plan = request.get('plan')

            task.assign_robots(robot_ids)

            task_schedule = self.resource_manager.get_task_schedule(task_id, robot_ids[0])
            task.update_schedule(task_schedule)

            self.logger.debug("Task %s was allocated to %s. Start navigation time: %s Finish time: %s", task.task_id,
                              [robot_id for robot_id in robot_ids],
                              task.start_time, task.finish_time)

            task.update_plan(robot_ids, task_plan)

            self.logger.debug('Task plan updated...')

        self.dispatcher.dispatch_tasks()
