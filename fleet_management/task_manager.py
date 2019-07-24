import logging

from ropod.utils.timestamp import TimeStamp as ts
from ropod.utils.uuid import generate_uuid

from ropod.structs.task import TaskRequest, Task
from ropod.structs.action import Action
from ropod.structs.status import TaskStatus, COMPLETED, TERMINATED, ONGOING, UNALLOCATED, ALLOCATED


from fleet_management.exceptions.task_allocator import UnsuccessfulAllocationAlternativeTimeSlot
from fleet_management.exceptions.osm_planner_exception import OSMPlannerException
from fleet_management.api import API


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
        self.api = api_config
        self.logger = logging.getLogger("fms.task.manager")

        self.logger.info("Task Manager initialized...")
        self.unallocated_tasks = dict()
        self.scheduled_tasks = list()

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
        """
        Dispatches all scheduled tasks that are ready for dispatching
        """
        for task_id, task in self.scheduled_tasks.items():
            if task_id not in self.ongoing_task_ids:
                if self.__can_execute_task(task_id):
                    self.logger.info('Dispatching task %s', task_id)
                    for robot_id, actions in task.robot_actions.items():
                        self.dispatch_task(task, robot_id)
                    self.ongoing_task_ids.append(task_id)
                    self.ccu_store.add_ongoing_task(task_id)
                    self.__initialise_task_status(task_id)
                    self.ccu_store.add_task_status(task.status)

    def dispatch_task(self, task, robot_id):
        """
        Sends a task to the appropriate robot fleet

        @param task a ropod.structs.task.Task object
        @param robot_id
        """
        self.logger.info("Dispatching task to robot %s", robot_id)
        task_msg = self.api.create_message(task, recipients=[robot_id])
        self.api.publish(task_msg)

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
        task.status.status = UNALLOCATED
        task.status.task_id = task.id
        self.task_statuses[task.id] = task.status

        self.logger.debug('Allocating robots for the task %s ', task.id)

        self.unallocated_tasks[task.id] = {'task': task,
                                           'plan': task_plan
                                           }

        self.resource_manager.get_robots_for_task(task)

    def process_task_requests(self):
            while self.resource_manager.allocations:
                task_id, robot_ids = self.resource_manager.allocations.pop()
                # for task_id, robot_ids in self.resource_manager.allocated_tasks.items():
                self.logger.warning('Reserving robots %s for task %s.', robot_ids, task_id)
                request = self.unallocated_tasks.pop(task_id)

                task = request.get('task')
                task_plan = request.get('plan')

                task.status.status = ALLOCATED
                task.team_robot_ids = robot_ids
                task_schedule = self.resource_manager.get_task_schedule(task_id, robot_ids[0])
                task.start_time = task_schedule['start_time']
                task.finish_time = task_schedule['finish_time']

                self.logger.info("Task %s was allocated to %s. Start time: %s Finish time: %s", task.id, [robot_id for robot_id in robot_ids],
                                 task.start_time, task.finish_time)
                for robot_id in robot_ids:
                    task.robot_actions[robot_id] = task_plan

                self.logger.debug('Saving task...')
                self.scheduled_tasks[task.id] = task
                self.ccu_store.add_task(task)
                self.logger.debug('Tasks saved')

    def suggest_alternative_timeslot(self, alternative_timeslots):
        """ Tasks in alternative_timeslots could not be allocated in the desired time window.
        Suggest a different start time for the task
        """
        for task_id, alternative_timeslot in alternative_timeslots.items():
            task_alternative_timeslot = dict()
            task_alternative_timeslot['header'] = dict()
            task_alternative_timeslot['payload'] = dict()
            task_alternative_timeslot['header']['type'] = 'TASK-ALTERNATIVE-TIMESLOT'
            task_alternative_timeslot['header']['metamodel'] = 'ropod-msg-schema.json'
            task_alternative_timeslot['header']['msgId'] = generate_uuid()
            task_alternative_timeslot['header']['timestamp'] = ts.get_time_stamp()
            task_alternative_timeslot['payload']['metamodel'] = 'ropod-task_alternative_timeslot-schema.json'
            task_alternative_timeslot['payload']['robot_id'] =  alternative_timeslot['robot_id']
            task_alternative_timeslot['payload']['task_id'] = task_id
            task_alternative_timeslot['payload']['start_time'] = alternative_timeslot['start_time']
            self.api.publish(task_alternative_timeslot)

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
        self.task_statuses[task_id] = task_status

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
