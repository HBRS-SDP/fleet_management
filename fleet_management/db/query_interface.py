from __future__ import print_function
import sys
import os.path
import pymongo as pm
import json
import time

from fleet_management.config.config_file_reader import ConfigFileReader
from ropod.pyre_communicator.base_class import RopodPyre
from ropod.utils.models import MessageFactory

class FleetManagementQueryInterface(RopodPyre):
    '''An interface for querying a fleet management database

    :groups: list of string (group names in pyre network)
    :db_name: string (name of database to be queried)
    :db_port: int (port on which mongodb responds)

    '''
    def __init__(self, groups, db_name='ropod_ccu_store', db_port=27017):
        super(FleetManagementQueryInterface, self).__init__(
                'ccu_query_interface', groups, list(), verbose=False)
        self.db_port = db_port
        self.db_name = db_name
        self.message_factory = MessageFactory()
        self.start()

    def zyre_event_cb(self, zyre_msg):
        '''Listens to "SHOUT" and "WHISPER" messages and returns a response
        if the incoming message is a query message 
        '''
        if zyre_msg.msg_type in ("SHOUT", "WHISPER"):
            response_msg = self.receive_msg_cb(zyre_msg.msg_content)
            if response_msg:
                self.whisper(response_msg, zyre_msg.peer_uuid)
                # print(response_msg)

    def receive_msg_cb(self, msg):
        '''Processes requests for queries;
        returns a dictionary representing a JSON response message

        Only listens to 
        "GET-ALL-ONGOING-TASKS", 
        "GET-ALL-SCHEDULED-TASKS", 
        "GET-ALL-SCHEDULED-TASK-IDS", 
        "GET-ROBOTS-ASSIGNED-TO-TASK" and 
        "GET-TASKS-ASSIGNED-TO-ROBOT"
        ignores all other messages (i.e. returns no value in such cases)

        :msg: string (a message in JSON format)

        '''
        dict_msg = self.convert_zyre_msg_to_dict(msg)
        if dict_msg is None:
            return

        if 'header' not in dict_msg or 'payload' not in dict_msg or 'type' not \
                in dict_msg['header'] or 'senderId' not in dict_msg['payload'] :
                    return None
        message_type = dict_msg['header']['type']
        receiverId = dict_msg['payload']['senderId']

        if message_type == "GET-ALL-ONGOING-TASKS" :
            ongoing_tasks = dict()

            db_client = pm.MongoClient(port=self.db_port)
            db = db_client[self.db_name]
            ongoing_task_collection = db['ongoing_tasks']
            task_collection = db['tasks']
            for task_dict in ongoing_task_collection.find():
                task_id = task_dict['task_id']
                ongoing_tasks[task_id] = task_collection.find(filter={"id":task_id})[0]

            return self.message_factory.get_query_msg(
                    message_type, 'tasks', ongoing_tasks, True, receiverId)

        elif message_type == "GET-ALL-SCHEDULED-TASKS" :
            scheduled_tasks = dict()

            db_client = pm.MongoClient(port=self.db_port)
            db = db_client[self.db_name]
            task_collection = db['tasks']
            for task_dict in task_collection.find():
                task_id = task_dict['id']
                scheduled_tasks[task_id] = task_dict

            return self.message_factory.get_query_msg(
                    message_type, 'tasks', scheduled_tasks, True, receiverId)

        elif message_type == "GET-ALL-SCHEDULED-TASK-IDS" :
            scheduled_task_ids = list()

            db_client = pm.MongoClient(port=self.db_port)
            db = db_client[self.db_name]
            task_collection = db['tasks']
            for task_dict in task_collection.find():
                scheduled_task_ids.append(task_dict['id'])

            return self.message_factory.get_query_msg(
                    message_type, 'taskIds', scheduled_task_ids, True, receiverId)

        elif message_type == "GET-ROBOTS-ASSIGNED-TO-TASK" : 
            if 'taskId' not in dict_msg['payload'] :
                return None
            task_id = dict_msg['payload']['taskId']
            robots = dict()

            db_client = pm.MongoClient(port=self.db_port)
            db = db_client[self.db_name]
            task_collection = db['tasks']
            robot_collection = db['robots']
            task_cursor = task_collection.find(filter={"id":task_id})
            if task_cursor.count() > 0 :
                task_dict = task_cursor.next()
                robot_ids = task_dict['team_robot_ids']
                for robot_id in robot_ids:
                    robots[robot_id] = robot_collection.find(filter={"robotId":robot_id})[0]
                success = True
            else :
                robots = []
                success = False

            return self.message_factory.get_query_msg(
                    message_type, 'robots', robots, success, receiverId)

        elif message_type == "GET-TASKS-ASSIGNED-TO-ROBOT" :
            if 'robotId' not in dict_msg['payload'] :
                return None
            robot_id = dict_msg['payload']['robotId']
            assigned_tasks = dict()

            db_client = pm.MongoClient(port=self.db_port)
            db = db_client[self.db_name]
            task_collection = db['tasks']
            for task_dict in task_collection.find():
                if robot_id in task_dict['team_robot_ids'] :
                    task_id = task_dict['id']
                    assigned_tasks[task_id] = task_dict

            return self.message_factory.get_query_msg(
                    message_type, 'tasks', assigned_tasks, True, receiverId)

        else :
            print(message_type, "is not a valid query type")


if __name__ == "__main__":
    code_dir = os.path.abspath(os.path.dirname(__file__))
    fms_dir = os.path.dirname(code_dir)
    main_dir = os.path.dirname(fms_dir)
    config_file = os.path.join(main_dir, "config/ccu_config.yaml")

    if len(sys.argv) < 2 :
        print("USAGE: python3 query_interface.py CONFIG_FILE.yaml")
        print("No config file provided. Using default config file")
    else :
        config_file = sys.argv[1]

    config_params = ConfigFileReader.load(config_file)
    # print(dir(config_params))
    query_interface = FleetManagementQueryInterface(
            ['ROPOD'], config_params.ccu_store_db_name)
    print('FleetManagement Query interface initialised')

    try:
        while True:
            time.sleep(0.5)
    except (KeyboardInterrupt, SystemExit):
        query_interface.shutdown()
        print('Query interface interrupted; exiting')
