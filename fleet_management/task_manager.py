import logging

from ropod.pyre_communicator.base_class import RopodPyre
from ropod.utils.timestamp import TimeStamp as ts
from ropod.utils.uuid import generate_uuid
from ropod.utils.models import MessageFactory

from ropod.structs.task import TaskRequest, Task
from ropod.structs.action import Action
from ropod.structs.status import TaskStatus, COMPLETED, TERMINATED, ONGOING

from fleet_management.task_planner_interface import TaskPlannerInterface
from fleet_management.resource_manager import ResourceManager
from fleet_management.path_planner import FMSPathPlanner


from fleet_management.db.init_db import initialize_robot_db, initialize_knowledge_base
from OBL import OSMBridge


class TaskManager(RopodPyre):
    '''An interface for handling ropod task requests and managing ropod tasks

    @author Alex Mitrevski
    @maintainer Alex Mitrevski, Argentina Ortega Sainz
    @contact aleksandar.mitrevski@h-brs.de, argentina.ortega@h-brs.de
    '''
    def __init__(self, ccu_store, api_config, plugins=[]):
        zyre_config = api_config.get('zyre')  # Arguments for the zyre_base class
        message_version = zyre_config.pop('message_version')

        super().__init__(zyre_config, message_version=message_version)

        self.scheduled_tasks = dict()
        self.ongoing_task_ids = list()
        self.task_statuses = dict()
        self.ccu_store = ccu_store
        self.logger = logging.getLogger("fms.task.manager")

        # TODO This is being used temporarily for testing, checks should be in place to
        # avoid overwriting existing data
        initialize_robot_db(config_params)

        # we initialize the knowledge base with some common knowledge,
        # such as the locations of the elevators in the environment
        initialize_knowledge_base(config_params.planner_params.kb_database_name)

        try:
            osm_bridge = OSMBridge(server_ip=config_params.overpass_server.ip, server_port=config_params.overpass_server.port)
        except Exception as e:
            self.logger.error("There is a problem in connecting to Overpass server. Error: %s", e)
            osm_bridge = None

        self.resource_manager = ResourceManager(config_params, ccu_store, osm_bridge)
        self.task_planner = TaskPlannerInterface(config_params.planner_params)
        self.path_planner = FMSPathPlanner(config_params=config_params, osm_bridge=osm_bridge)
        self.logger.info("Task Manager initialized...")

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

    def receive_msg_cb(self, msg_content):
        '''Processes a task request message; ignores all other messages.
        Only responds to messages of type TASK-REQUEST and TASK-PROGRESS

        @param msg_content a dictionary representing a Zyre message
        '''
        dict_msg = self.convert_zyre_msg_to_dict(msg_content)
        if dict_msg is None:
            return

        # NOTE: A task request should now contain Area names (not SubArea!)
        message_type = dict_msg['header']['type']
        if message_type == 'TASK-REQUEST':
            self.logger.debug('Received a task request; processing request')
            payload = dict_msg['payload']

            task_request = TaskRequest.from_dict(payload)

            # TODO get_area function should also return floor number
            task_request.pickup_pose = self.path_planner.get_area(payload['pickupLocation'])
            task_request.pickup_pose.floor_number = payload['pickupLocationLevel']

            task_request.delivery_pose = self.path_planner.get_area(payload['deliveryLocation'])
            task_request.delivery_pose.floor_number = payload['deliveryLocationLevel']

            self.__process_task_request(task_request)

        elif message_type == 'TASK-PROGRESS':

            action_type = dict_msg['payload']['actionType']
            self.logger.debug("Received task progress message... Action %s %s " % (dict_msg["payload"]["actionType"],
                                                                       dict_msg["payload"]['status']["areaName"]))
            task_id = dict_msg["payload"]["taskId"]
            robot_id = dict_msg["payload"]["robotId"]
            current_action = dict_msg["payload"]["actionId"]
            action_type = dict_msg["payload"]["actionType"]
            area_name = dict_msg["payload"]["status"]["areaName"]
            action_status = dict_msg["payload"]["status"]["actionStatus"]
            if action_type == "GOTO":
                current_action = dict_msg["payload"]["status"]["sequenceNumber"]
                total_actions = dict_msg["payload"]["status"]["totalNumber"]

            task_status = dict_msg["payload"]["status"]["taskStatus"]

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
        super().shutdown()
        self.resource_manager.shutdown()

    def start(self):
        super().start()
        self.resource_manager.start()
