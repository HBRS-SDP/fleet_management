from __future__ import print_function
import sys
import time
import uuid
import os.path
from fleet_management.config.config_file_reader import ConfigFileReader
from ropod.pyre_communicator.base_class import RopodPyre
from fleet_management.db.ccu_store import CCUStore

class FleetManagementQueryInterface(RopodPyre):
    '''An interface for querying a fleet management database

    '''
    def __init__(self, groups, db_name='ropod_ccu_store', db_port=27017):
        super(FleetManagementQueryInterface, self).__init__(
                'ccu_query_interface', groups, list(), verbose=True)
        self.ccu_store = CCUStore(db_name, db_port)
        self.start()

    def zyre_event_cb(self, zyre_msg):
        '''Listens to "SHOUT" and "WHISPER" messages and returns a response
        if the incoming message is a query message 
        '''
        if zyre_msg.msg_type in ("SHOUT", "WHISPER"):
            response_msg = self.receive_msg_cb(zyre_msg.msg_content)
            if response_msg:
                self.whisper(response_msg, zyre_msg.peer_uuid)
                print(response_msg)

    def receive_msg_cb(self, msg):
        '''Processes requests for queries;
        returns a dictionary representing a JSON response message

        Only listens to 
        "QUERY-ALL-ONGOING-TASKS", 
        "QUERY-ALL-SCHEDULED-TASKS", 
        "QUERY-ROBOTS-ASSIGNED-TO-TASK" and 
        "QUERY-TASKS-ASSIGNED-TO-ROBOT"
        ignores all other messages (i.e. returns no value in such cases)

        @param msg a message in JSON format
        '''
        dict_msg = self.convert_zyre_msg_to_dict(msg)
        if dict_msg is None:
            return

        print(dict_msg)
        message_type = dict_msg['header']['type']
        if message_type == "QUERY-ALL-ONGOING-TASKS" :
            response = self.ccu_store.get_ongoing_tasks()
            response_msg = self.__get_response_msg_skeleton(message_type)
            response_msg['payload']['taskIds'] = response
            return response_msg
        elif message_type == "QUERY-ALL-SCHEDULED-TASKS" :
            response = self.ccu_store.get_scheduled_tasks()
            response = list(response.keys())
            response_msg = self.__get_response_msg_skeleton(message_type)
            response_msg['payload']['taskIds'] = response
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
        @param msg_type a string representing a message type

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
        print('[{0}] Query interface interrupted; exiting'.format(config_params.bb_id))
