from __future__ import print_function
import sys
import time
import uuid
import os.path
import pymongo as pm
import json
from fleet_management.config.config_file_reader import ConfigFileReader
from ropod.pyre_communicator.base_class import RopodPyre

class FleetManagementQueryInterface(RopodPyre):
    '''An interface for querying a fleet management database

    :groups: list of string (group names in pyre network)
    :db_name: string (name of database to be queried)
    :db_port: int (port on which mongodb responds)

    '''
    def __init__(self, groups, db_name='ropod_ccu_store', db_port=27017):
        super(FleetManagementQueryInterface, self).__init__(
                'ccu_query_interface', groups, list(), verbose=True)
        self.db_port = db_port
        self.db_name = db_name
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
        "QUERY-ALL-ONGOING-TASKS", 
        "QUERY-ALL-SCHEDULED-TASKS", 
        "QUERY-ROBOTS-ASSIGNED-TO-TASK" and 
        "QUERY-TASKS-ASSIGNED-TO-ROBOT"
        ignores all other messages (i.e. returns no value in such cases)

        :msg: string (a message in JSON format)

        '''
        dict_msg = self.convert_zyre_msg_to_dict(msg)
        if dict_msg is None:
            return

        message_type = dict_msg['header']['type']
        if message_type == "QUERY-ALL-ONGOING-TASKS" :
            db_client = pm.MongoClient(port=self.db_port)
            db = db_client[self.db_name]
            ongoing_task_collection = db['ongoing_tasks']
            task_collection = db['tasks']

            ongoing_tasks = dict()
            for task_dict in ongoing_task_collection.find():
                task_id = task_dict['task_id']
                ongoing_tasks[task_id] = task_collection.find(filter={"id":task_id})[0]

            response = json.dumps(ongoing_tasks, indent=2, default=str)
            response_msg = self.__get_response_msg_skeleton(message_type)
            response_msg['payload']['tasks'] = response
            return response_msg

        elif message_type == "QUERY-ALL-SCHEDULED-TASKS" :
            db_client = pm.MongoClient(port=self.db_port)
            db = db_client[self.db_name]
            collection = db['tasks']

            scheduled_tasks = dict()
            for task_dict in collection.find():
                task_id = task_dict['id']
                scheduled_tasks[task_id] = task_dict

            response = json.dumps(scheduled_tasks, indent=2, default=str)
            response_msg = self.__get_response_msg_skeleton(message_type)
            response_msg['payload']['tasks'] = response
            return response_msg

        else :
            print(message_type, "is not a valid query type")

    def __get_response_msg_skeleton(self, msg_type):
        '''Returns a dictionary representing a query response for the given message type.
        The dictionary has the following format:
        {
            "header":
            {
                "metamodel": "ropod-msg-schema.json",
                "type": msg_type,
                "msgId": message-uuid,
                "timestamp": current-time
            },
            "payload":
            {
                "receiverId": ""
            }
        }

        Keyword arguments:
        :msg_type: a string representing a message type

        '''
        response_msg = dict()
        response_msg['header'] = dict()
        response_msg['header']['metamodel'] = 'ropod-msg-schema.json'
        response_msg['header']['type'] = msg_type
        response_msg['header']['msgId'] = str(uuid.uuid4())
        response_msg['header']['timestamp'] = time.time()
        response_msg['payload'] = dict()
        return response_msg

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
