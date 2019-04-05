import logging

from ropod.utils.timestamp import TimeStamp as ts
from ropod.utils.uuid import generate_uuid

from ropod.structs.task import TaskRequest, Task
from ropod.structs.action import Action
from ropod.structs.status import TaskStatus, COMPLETED, TERMINATED, ONGOING


class TaskManager(object):
    '''An interface for handling ropod task requests and managing ropod tasks

    @author Alex Mitrevski
    @maintainer Alex Mitrevski, Argentina Ortega Sainz
    @contact aleksandar.mitrevski@h-brs.de, argentina.ortega@h-brs.de
    '''
    def __init__(self, ccu_store, api_config, plugins=[]):
        self.scheduled_tasks = dict()
        self.ongoing_task_ids = list()
        self.task_statuses = dict()
        self.ccu_store = ccu_store
        self.logger = logging.getLogger("fms.task.manager")

        self.logger.info("Task Manager initialized...")

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
        # TODO this needs to be updated since the new config file format removed some data
        self.resource_manager.restore_data()

    def task_request_cb(self, msg):
        payload = msg['payload']

        task_request = TaskRequest.from_dict(payload)

        # TODO get_area function should also return floor number
        task_request.pickup_pose = self.path_planner.get_area(payload['pickupLocation'])
        task_request.pickup_pose.floor_number = payload['pickupLocationLevel']

        task_request.delivery_pose = self.path_planner.get_area(payload['deliveryLocation'])
        task_request.delivery_pose.floor_number = payload['deliveryLocationLevel']

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

    def dispatch_tasks(self):
        """Dispatches all scheduled tasks that are ready for dispatching
        """
        for task_id, task in self.scheduled_tasks.items():
            if task_id not in self.ongoing_task_ids:
                if self.__can_execute_task(task_id):
                    current_time = ts.get_time_stamp()
                    self.logger.info('[%s] Dispatching task %s', current_time, task_id)
                    self.dispatch_task(task)
                    self.ongoing_task_ids.append(task_id)
                    self.ccu_store.add_ongoing_task(task_id)
                    self.__initialise_task_status(task_id)
                    self.ccu_store.add_task_status(task.status)

    def dispatch_task(self, task):
        '''Sends a task to the appropriate robot fleet

        @param task a ropod.structs.task.Task object
        '''
        self.logger.info("Dispatching task: %s ", task.id)
        for robot_id, actions in task.robot_actions.items():

            msg = self.mf.create_message(task, recipients=[robot_id])
            self.shout(msg)

    def __can_execute_task(self, task_id):
        '''Returns True if the given task needs to be dispatched
        based on the task schedule; returns False otherwise

        @param task_id UUID representing the ID of a task

        '''
        current_time = ts.get_time_stamp()
        task_start_time = self.scheduled_tasks[task_id].start_time
        if task_start_time < current_time:
            return True
        return False

    def __process_task_request(self, request):
        '''Processes a task request, namely chooses robots for the task
        and generates an appropriate task plan

        @param request a ropod.structs.task.TaskRequest object
        '''
        # TODO save all received task requests to the ccu_store
        self.logger.debug('Creating a task plan...')
        task_plan = self.task_planner.get_task_plan_without_robot(request, self.path_planner)
        for action in task_plan:
            action.id = generate_uuid()

        self.logger.debug('Creating a task...')
        task = Task.from_request(request)

        self.logger.debug('Allocating robots for the task...')
        allocation = self.resource_manager.get_robots_for_task(task)
        task.status.status = "allocated"

        for task_id, robot_ids in allocation.items():
            task.team_robot_ids = robot_ids
            task_schedule = self.resource_manager.get_tasks_schedule_robot(task_id, robot_ids[0])
            task.start_time = task_schedule['start_time']
            task.finish_time = task_schedule['finish_time']

        for task_id, robot_ids in allocation.items():
            self.logger.info("Task %s was allocated to %s", task.id, [robot_id for robot_id in robot_ids])
            for robot_id in robot_ids:
                task.robot_actions[robot_id] = task_plan

        self.logger.debug('Saving task...')
        self.scheduled_tasks[task.id] = task
        self.ccu_store.add_task(task)
        self.logger.debug('Task saved')

    def __initialise_task_status(self, task_id):
        '''Called after task task_allocation. Sets the task status for the task with ID 'task_id' to "ongoing"

        @param task_id UUID representing the ID of a task
        '''
        task = self.scheduled_tasks[task_id]
        # TODO this task status is not being used.
        task_status = TaskStatus()
        task_status.task_id = task_id
        task_status.status = ONGOING
        for robot_id in task.team_robot_ids:
            task.status.current_robot_action[robot_id] = task.robot_actions[robot_id][0].id
            task.status.completed_robot_actions[robot_id] = list()
            task.status.estimated_task_duration = task.estimated_duration
        self.task_statuses[task_id] = task.status

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
        status = self.task_statuses[task_id]
        status.status = task_status
        if task_status == TERMINATED or task_status == COMPLETED:
            if task_status == TERMINATED:
                self.logger.debug("Task terminated")
            elif task_status == COMPLETED:
                self.logger.debug("Task completed!")
            task = self.scheduled_tasks[task_id]
            self.ccu_store.archive_task(task, task.status)
            self.scheduled_tasks.pop(task_id)
            self.task_statuses.pop(task_id)
            if task_id in self.ongoing_task_ids:
                self.ongoing_task_ids.remove(task_id)
        elif task_status == ONGOING:
            previous_action = status.current_robot_action[robot_id]
            status.completed_robot_actions[robot_id].append(previous_action)
            status.current_robot_action[robot_id] = current_action
            self.ccu_store.update_task_status(status)

            # TODO: update the estimated time duration based on the current timestamp
            # and the estimated duration of the rest of the tasks

    def __get_action(self, task_id, robot_id, action_id):
        '''Returns the action with ID 'action_id' that is part of the task with ID 'task_id'
        and is performed by the robot with ID 'robot_id'

        @param task_id UUID representing a scheduled task
        @param robot_id name of a robot
        @param action_id UUID representing a task
        '''
        task = self.scheduled_tasks[task_id]
        desired_action = Action()
        for action in task.actions[robot_id]:
            if action.id == action_id:
                desired_action = action
                break
        return desired_action

    def shutdown(self):
        self.resource_manager.shutdown()

    def start(self):
        self.resource_manager.start()
