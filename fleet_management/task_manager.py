import logging

from fleet_management.exceptions.osm_planner_exception import OSMPlannerException
from ropod.structs.status import TaskStatus
from ropod.structs.task import TaskRequest, Task
from ropod.utils.uuid import generate_uuid


class TaskManager(object):
    '''An interface for handling ropod task requests and managing ropod tasks

    @author Alex Mitrevski
    @maintainer Alex Mitrevski, Argentina Ortega Sainz
    @contact aleksandar.mitrevski@h-brs.de, argentina.ortega@h-brs.de
    '''
    def __init__(self, ccu_store, api_config, plugins=[]):
        self.ongoing_task_ids = list()
        self.task_statuses = dict()
        self.ccu_store = ccu_store
        self.api = api_config
        self.logger = logging.getLogger("fms.task.manager")

        self.logger.info("Task Manager initialized...")
        self.unallocated_tasks = dict()

    def add_plugin(self, name, obj):
        self.__dict__[name] = obj
        self.logger.debug("Added %s plugin to %s", name, self.__class__.__name__)

    def __str__(self):
        return str(self.__dict__)

    def get_scheduled_tasks(self):
        '''Returns a dictionary of all scheduled tasks
        '''
        return self.scheduled_tasks

    def get_ongoing_tasks_ids(self):
        '''Returns a list of the task IDs of ongoing tasks
        '''
        return self.ongoing_task_ids

    def get_ongoing_task_statuses(self):
        '''Returns a dictionary containing the statuses of the ongoing tasks
        '''
        return self.task_statuses

    def restore_task_data(self):
        '''Loads any existing task data (ongoing tasks, scheduled tasks) from the CCU store database
        '''
        self.scheduled_tasks = self.ccu_store.get_scheduled_tasks()
        self.ongoing_task_ids = self.ccu_store.get_ongoing_tasks()
        self.task_statuses = self.ccu_store.get_ongoing_task_statuses()
        self.resource_manager.restore_data()

    def task_request_cb(self, msg):
        payload = msg['payload']

        task_request = TaskRequest.from_dict(payload)

        # TODO get_area function should also return floor number
        task_request.pickup_pose = self.path_planner.get_area(
                payload['pickupLocation'],
                get_level=True)

        task_request.delivery_pose = self.path_planner.get_area(
                payload['deliveryLocation'],
                get_level=True)

        self.logger.debug("Processing task request")
        self.__process_task_request(task_request)

    def task_progress_cb(self, msg):
        action_type = msg['payload']['actionType']
        self.logger.debug("Received task progress message... Action %s %s " % (msg["payload"]["actionType"],
                                                                               msg["payload"]['status']["areaName"]))
        task_id = msg["payload"]["taskId"]
        robot_id = msg["payload"]["robotId"]
        current_action = msg["payload"]["actionId"]
        action_type = msg["payload"]["actionType"]
        area_name = msg["payload"]["status"]["areaName"]
        action_status = msg["payload"]["status"]["actionStatus"]
        if action_type == "GOTO":
            current_action = msg["payload"]["status"]["sequenceNumber"]
            total_actions = msg["payload"]["status"]["totalNumber"]

        task_status = msg["payload"]["status"]["taskStatus"]

        self.__update_task_status(task_id, robot_id, current_action, task_status)

    def __process_task_request(self, request):
        '''Processes a task request, namely chooses robots for the task
        and generates an appropriate task plan

        @param request a ropod.structs.task.TaskRequest object
        '''
        # TODO save all received task requests to the ccu_store
        self.logger.debug('Creating a task plan...')
        try:
            task_plan = self.task_planner.get_task_plan_without_robot(request, self.path_planner)
            # TODO Add a request ID from message
            request_id = 111
            self.logger.debug('Planning successful for task %s', request_id)
        except OSMPlannerException as e:
            self.logger.error("Can't process task request")
            self.logger.error(str(e))
            return  # TODO: this error needs to be communicated with the end user

        for action in task_plan:
            action.id = generate_uuid()

        self.logger.debug('Creating a task for request %s ', request_id)
        task = Task.from_request(request)
        self.logger.debug('Created task %s for request %s', task.id, request_id)
        # Assuming a constant velocity of 1m/s, the estimated duration of the task is the
        # distance from the pickup to the delivery pose
        self.logger.debug("Estimating task duration between %s and %s, from floor %s to %s....",
                          request.pickup_pose.name, request.delivery_pose.name, request.pickup_pose.floor_number,
                          request.delivery_pose.floor_number)
        estimated_duration = self.path_planner.get_estimated_path_distance(request.pickup_pose.floor_number,
                                                                           request.delivery_pose.floor_number,
                                                                           request.pickup_pose.name,
                                                                           request.delivery_pose.name)
        self.logger.debug('Estimated duration for the task: %s', estimated_duration)

        task.update_task_estimated_duration(estimated_duration)
        # task.status.status = TaskStatus.UNALLOCATED
        task.status.task_id = task.id
        self.task_statuses[task.id] = task.status

        self.logger.debug('Allocating robots for the task %s ', task.id)

        self.unallocated_tasks[task.id] = {'task': task,
                                           'plan': task_plan
                                           }

        self.resource_manager.get_robots_for_task(task)
        self.logger.error('Sent to resource manager for allocation')

    def process_task_requests(self):

        while self.resource_manager.allocations:
            task_id, robot_ids = self.resource_manager.allocations.pop()
            self.logger.warning('Reserving robots %s for task %s.', robot_ids, task_id)
            request = self.unallocated_tasks.pop(task_id)

            task = request.get('task')
            task_plan = request.get('plan')

            task.team_robot_ids = robot_ids

            for robot_id in robot_ids:
                task.robot_actions[robot_id] = task_plan

    def __update_task_status(self, task_id, robot_id, current_action, task_status):
        '''Updates the status of the robot with ID 'robot_id' that is performing
        the task with ID 'task_id'

        If 'task_status' is "terminated", removes the task from the list of scheduled
        and ongoing tasks and saves a historical database entry for the task.
        On the other hand, if 'task_status' is "ongoing", the task's entry
        is updated for the appropriate robot.

        @param task_id UUID representing a previously scheduled task
        @param robot_id name of a robot
        @param current_action UUID representing an action
        @param task_status a string representing the status of a task;
               takes the values "unallocated", "allocated", "ongoing", "terminated", and "completed"
        '''
        self.logger.debug("New task status: %s ", task_status)
        status = self.task_statuses[task_id]
        self.logger.debug("Previous task status: %s ", status.status)
        status.status = task_status

        if task_status == TaskStatus.CANCELED or task_status == TaskStatus.COMPLETED:
            if task_status == TaskStatus.CANCELED:
                self.logger.debug("Task terminated")
            elif task_status == TaskStatus.COMPLETED:
                self.logger.debug("Task completed!")
            task = self.scheduled_tasks[task_id]
            self.ccu_store.archive_task(task, task.status)
            self.scheduled_tasks.pop(task_id)
            self.task_statuses.pop(task_id)
            if task_id in self.ongoing_task_ids:
                self.ongoing_task_ids.remove(task_id)
        elif task_status == TaskStatus.ONGOING:
            previous_action = status.current_robot_action[robot_id]
            status.completed_robot_actions[robot_id].append(previous_action)
            status.current_robot_action[robot_id] = current_action
            self.ccu_store.update_task_status(status)

            # TODO: update the estimated time duration based on the current timestamp
            # and the estimated duration of the rest of the tasks
