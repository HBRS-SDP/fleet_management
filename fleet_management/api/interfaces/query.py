import logging
import sys
import time

import pymongo as pm
from ropod.pyre_communicator.base_class import RopodPyre
from ropod.utils.models import RopodMessageFactory
from ropod.structs.status import TaskStatus

from fmlib.models.tasks import Task
from fmlib.models.robot import Robot

class QueryInterface(RopodPyre):
    """An interface for querying a fleet management database

    :groups: list of string (group names in pyre network)
    :db_name: string (name of database to be queried)
    :db_port: int (port on which mongodb responds)

    """

    def __init__(self, zyre_config, db_name='ropod_ccu_store', db_port=27017):
        super(QueryInterface, self).__init__(zyre_config)

        self.logger = logging.getLogger('fms.interfaces.query')
        self.db_port = db_port
        self.db_name = db_name
        self.message_factory = RopodMessageFactory()
        self.start()
        self._msg_to_func = {
            "GET-ALL-ONGOING-TASKS": self.__get_all_ongoing_tasks,
            "GET-ALL-SCHEDULED-TASKS": self.__get_all_scheduled_tasks,
            "GET-ALL-SCHEDULED-TASK-IDS": self.__get_all_scheduled_task_ids,
            "GET-ROBOTS-ASSIGNED-TO-TASK": self.__get_robots_assigned_to_task,
            "GET-TASKS-ASSIGNED-TO-ROBOT": self.__get_tasks_assigned_to_robot,
            "GET-ROBOT-STATUS": self.__get_robot_status
        }

    def zyre_event_cb(self, zyre_msg):
        """Listens to "SHOUT" and "WHISPER" messages and returns a response
        if the incoming message is a query message
        """
        if zyre_msg.msg_type in ("SHOUT", "WHISPER"):
            response_msg = self.receive_msg_cb(zyre_msg.msg_content)
            if response_msg:
                self.whisper(response_msg, zyre_msg.peer_uuid)

    def receive_msg_cb(self, msg):
        """Processes requests for queries;
        returns a dictionary representing a JSON response message

        Only listens to messages of type defined in _msg_to_func
        ignores all other messages (i.e. returns no value in such cases)

        :msg: string (a message in JSON format)

        """
        dict_msg = self.convert_zyre_msg_to_dict(msg)
        if dict_msg is None:
            return

        if 'header' not in dict_msg or 'payload' not in dict_msg or 'type' not \
                in dict_msg['header'] or 'senderId' not in dict_msg['payload']:
            return None
        message_type = dict_msg['header']['type']
        receiverId = dict_msg['payload']['senderId']

        if message_type in self._msg_to_func:
            return self._msg_to_func[message_type](dict_msg)
        else:
            # self.logger.warning(str(message_type)+ "is not a valid query type")
            return None

    def __get_robot_status(self, dict_msg):
        message_type = dict_msg['header']['type']
        receiverId = dict_msg['payload']['senderId']
        if 'robotId' not in dict_msg['payload']:
            return None
        robot_id = dict_msg['payload']['robotId']
        status = dict()
        success = True

        try:
            robot = Robot.get_robot(robot_id)
            status[robot_id] = {} # TODO
        except Exception as e:
            success = False

        return self.message_factory.get_query_msg(
            message_type, 'status', status, success, receiverId)

    def __get_all_ongoing_tasks(self, dict_msg):
        message_type = dict_msg['header']['type']
        receiverId = dict_msg['payload']['senderId']
        ongoing_tasks = dict()

        tasks = Task.get_tasks_by_status(TaskStatus.ONGOING)
        for task in tasks:
            task_dict = task.to_dict()
            task_id = task_dict['task_id']
            ongoing_tasks[task_id] = task_dict

        return self.message_factory.get_query_msg(
            message_type, 'tasks', ongoing_tasks, True, receiverId)

    def __get_all_scheduled_tasks(self, dict_msg):
        message_type = dict_msg['header']['type']
        receiverId = dict_msg['payload']['senderId']
        scheduled_tasks = dict()

        tasks = Task.get_tasks_by_status(TaskStatus.SCHEDULED)
        for task in tasks:
            task_dict = task.to_dict()
            task_id = task_dict['task_id']
            scheduled_tasks[task_id] = task_dict

        return self.message_factory.get_query_msg(
            message_type, 'tasks', scheduled_tasks, True, receiverId)

    def __get_all_scheduled_task_ids(self, dict_msg):
        message_type = dict_msg['header']['type']
        receiverId = dict_msg['payload']['senderId']
        scheduled_task_ids = list()

        tasks = Task.get_tasks_by_status(TaskStatus.SCHEDULED)
        for task in tasks:
            task_dict = task.to_dict()
            task_id = task_dict['task_id']
            scheduled_task_ids.append(task_id)

        return self.message_factory.get_query_msg(
            message_type, 'taskIds', scheduled_task_ids, True, receiverId)

    def __get_robots_assigned_to_task(self, dict_msg):
        message_type = dict_msg['header']['type']
        receiverId = dict_msg['payload']['senderId']
        if 'taskId' not in dict_msg['payload']:
            return None
        task_id = dict_msg['payload']['taskId']
        robots = dict()

        task = None
        success = True
        try:
            task = Task.get_task(task_id)
            task_dict = task.to_dict()
            for robot_id in task_dict['assigned_robots']:
                robots[robot_id] = Robot.get_robot(robot_id)
        except Exception as e:
            success = False

        return self.message_factory.get_query_msg(
            message_type, 'robots', robots, success, receiverId)

    def __get_tasks_assigned_to_robot(self, dict_msg):
        message_type = dict_msg['header']['type']
        receiverId = dict_msg['payload']['senderId']
        if 'robotId' not in dict_msg['payload']:
            return None
        robot_id = dict_msg['payload']['robotId']
        assigned_tasks = dict()

        tasks = Task.get_tasks_by_robot(robot_id)
        for task in tasks:
            task_dict = task.to_dict()
            task_id = task_dict['task_id']
            assigned_tasks[task_id] = task_dict

        return self.message_factory.get_query_msg(
            message_type, 'tasks', assigned_tasks, True, receiverId)
