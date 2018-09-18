from __future__ import print_function

from pyre_communicator.base_class import PyreBaseCommunicator
from fleet_management.structs.task import TaskRequest, Task
from fleet_management.structs.action import Action
from fleet_management.structs.status import TaskStatus
from fleet_management.task_planner import TaskPlanner
from fleet_management.resource_manager import ResourceManager
from fleet_management.path_planner import PathPlanner


class TaskManager(PyreBaseCommunicator):
    '''An interface for handling ropod task requests and managing ropod tasks

    @author Alex Mitrevski
    @maintainer Alex Mitrevski, Argentina Ortega Sainz
    @contact aleksandar.mitrevski@h-brs.de, argentina.ortega@h-brs.de
    '''
    def __init__(self, config_params, ccu_store):
        super().__init__(config_params.task_manager_zyre_params.node_name,
                         config_params.task_manager_zyre_params.groups,
                         config_params.task_manager_zyre_params.message_types)
        self.scheduled_tasks = dict()
        self.ongoing_task_ids = list()
        self.task_statuses = dict()
        self.ccu_store = ccu_store
        self.resource_manager = ResourceManager(config_params, ccu_store)
        self.path_planner = PathPlanner(config_params.overpass_server)

    '''Returns a dictionary of all scheduled tasks
    '''
    def get_scheduled_tasks(self):
        return self.scheduled_tasks

    '''Returns a list of the task IDs of ongoing tasks
    '''
    def get_ongoing_tasks_ids(self):
        return self.ongoing_task_ids

    '''Returns a dictionary containing the statuses of the ongoing tasks
    '''
    def get_ongoing_task_statuses(self):
        return self.task_statuses

    '''Loads any existing task data (ongoing tasks, scheduled tasks) from the CCU store database
    '''
    def restore_task_data(self):
        self.scheduled_tasks = self.ccu_store.get_scheduled_tasks()
        self.ongoing_task_ids = self.ccu_store.get_ongoing_tasks()
        self.task_statuses = self.ccu_store.get_ongoing_task_statuses()
        self.resource_manager.restore_data()

    '''Processes a task request message; ignores all other messages.
    Only responds to messages of type TASK-REQUEST and TASK-PROGRESS

    @param msg_content a dictionary representing a Zyre message
    '''
    def receive_msg_cb(self, msg_content):
        dict_msg = self.convert_zyre_msg_to_dict(msg_content)
        if dict_msg is None:
            return

        message_type = dict_msg['header']['type']
        if message_type == 'TASK-REQUEST':
            print('Received a task request; processing request')
            user_id = dict_msg["payload"]["userId"]
            device_type = dict_msg["payload"]["deviceType"]
            device_id = dict_msg["payload"]["deviceId"]

            task_earliest_start_time = dict_msg["payload"]["earliestStartTime"]
            task_latest_start_time = dict_msg["payload"]["latestStartTime"]

            pickup_location = dict_msg["payload"]["pickupLocation"]
            pickup_location_level = dict_msg["payload"]["pickupLocationLevel"]

            delivery_location = dict_msg["payload"]["deliveryLocation"]
            delivery_location_level = dict_msg["payload"]["deliveryLocationLevel"]

            priority = dict_msg["payload"]["priority"]
            # TODO add priority, earliest_start_time and latest_start_time to the payload of json task request schema

            task_request = TaskRequest()
            task_request.user_id = user_id
            task_request.cart_type = device_type
            task_request.cart_id = device_id
            task_request.earliest_start_time = task_earliest_start_time
            task_request.latest_start_time = task_latest_start_time

            task_request.pickup_pose = self.path_planner.get_area(pickup_location)
            task_request.pickup_pose.floor_number = pickup_location_level

            task_request.delivery_pose = self.path_planner.get_area(delivery_location)
            task_request.delivery_pose.floor_number = delivery_location_level
            task_request.priority = priority
            self.__process_task_request(task_request)
        elif message_type == 'TASK-PROGRESS':
            task_id = dict_msg["payload"]["taskId"]
            robot_id = dict_msg["payload"]["robotId"]
            current_action = dict_msg["payload"]["status"]["currentAction"]
            task_status = dict_msg["payload"]["status"]["taskStatus"]
            self.__update_task_status(task_id, robot_id, current_action, task_status)

    '''Dispatches all scheduled tasks that are ready for dispatching
    '''
    def dispatch_tasks(self):
        for task_id, task in self.scheduled_tasks.items():
            if task_id not in self.ongoing_task_ids:
                if self.__can_execute_task(task_id):
                    current_time = self.get_time_stamp()
                    print('[{0}] Dispatching task {1}'.format(current_time, task_id))
                    self.dispatch_task(task)
                    self.ongoing_task_ids.append(task_id)
                    self.ccu_store.add_ongoing_task(task_id)
                    self.__initialise_task_status(task_id)
                    self.ccu_store.add_task_status(task.status)

    '''Sends a task to the appropriate robot fleet

    @param task a fleet_management.structs.task.Task object
    '''
    def dispatch_task(self, task):
        for robot_id, actions in task.actions.items():
            msg_dict = dict()
            msg_dict['header'] = dict()
            msg_dict['payload'] = dict()

            msg_dict['header']['type'] = 'TASK'
            msg_dict['header']['metamodel'] = 'ropod-msg-schema.json'
            msg_dict['header']['msgId'] = self.generate_uuid()
            msg_dict['header']['robotId'] = robot_id
            msg_dict['header']['timestamp'] = -1

            msg_dict['payload']['metamodel'] = 'ropod-task-schema.json'
            msg_dict['payload']['taskId'] = task.id
            msg_dict['payload']['teamRobotIds'] = task.team_robot_ids
            msg_dict['payload']['actions'] = list()
            for action in actions:
                action_dict = action.to_dict()
                msg_dict['payload']['actions'].append(action_dict)
            self.shout(msg_dict)

    '''Returns True if the given task needs to be dispatched
    based on the task schedule; returns False otherwise

    @param task_id UUID representing the ID of a task

    '''
    def __can_execute_task(self, task_id):
        current_time = self.get_time_stamp()
        task_start_time = self.scheduled_tasks[task_id].start_time
        if task_start_time < current_time:
            return True
        return False

    '''Processes a task request, namely chooses robots for the task
    and generates an appropriate task plan

    @param request a fleet_management.structs.task.TaskRequest object
    '''
    def __process_task_request(self, request):
        print('Creating a task plan...')
        task_plan = TaskPlanner.get_task_plan(request, self.path_planner)
        for action in task_plan:
            action.id = self.generate_uuid()

        print('Creating a task...')
        task = Task()
        task.id = self.generate_uuid()
        task.cart_type = request.cart_type
        task.cart_id = request.cart_id
        task.earliest_start_time = request.earliest_start_time
        task.latest_start_time = request.latest_start_time
        task.pickup_pose = request.pickup_pose
        task.delivery_pose = request.delivery_pose
        task.priority = request.priority
        task.status.status = "unallocated"
        task.status.task_id = task.id
        task.team_robot_ids = None
        print('Saving task...')
        self.scheduled_tasks[task.id] = task
        self.ccu_store.add_task(task)
        print('Task saved')

        print('Allocating robots for the task...')
        task_robots = self.resource_manager.get_robots_for_task(task)
        task.status.status = "allocated"

        task.team_robot_ids = task_robots
        for robot_id in task_robots:
            task.robot_actions[robot_id] = task_plan


    '''Called after task allocation. Sets the task status for the task with ID 'task_id' to "ongoing"

    @param task_id UUID representing the ID of a task
    '''
    def __initialise_task_status(self, task_id):
        task = self.scheduled_tasks[task_id]
        task.status.status = 'ongoing'
        for robot_id in task.team_robot_ids:
            task.status.current_robot_action[robot_id] = task.robot_actions[robot_id][0].id
            task.status.completed_robot_actions[robot_id] = list()
            task.status.estimated_task_duration = task.estimated_duration
        self.task_statuses[task_id] = task.status

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
    def __update_task_status(self, task_id, robot_id, current_action, task_status):
        task = self.scheduled_tasks[task_id]
        task.status.status = task_status
        if task_status == 'terminated' or task_status == 'completed':
            task = self.scheduled_tasks[task_id]
            self.ccu_store.archive_task(task, task.status)
            self.scheduled_tasks.pop(task_id)
            self.task_statuses.pop(task_id)
            if task_id in self.ongoing_task_ids:
                self.ongoing_task_ids.remove(task_id)
        elif task_status == 'ongoing':
            previous_action = task.status.current_robot_action[robot_id]
            task.status.completed_robot_actions[robot_id].append(previous_action)
            task.status.current_robot_action[robot_id] = current_action
            self.ccu_store.update_task_status(task.status)

            # TODO: update the estimated time duration based on the current timestamp
            # and the estimated duration of the rest of the tasks

    '''Returns the action with ID 'action_id' that is part of the task with ID 'task_id'
    and is performed by the robot with ID 'robot_id'

    @param task_id UUID representing a scheduled task
    @param robot_id name of a robot
    @param action_id UUID representing a task
    '''
    def __get_action(self, task_id, robot_id, action_id):
        task = self.scheduled_tasks[task_id]
        desired_action = Action()
        for action in task.actions[robot_id]:
            if action.id == action_id:
                desired_action = action
                break
        return desired_action

    def shutdown(self):
        super().shutdown()
        self.resource_manager.shutdown()
