import logging

from fleet_management.exceptions.osm_planner_exception import OSMPlannerException
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


