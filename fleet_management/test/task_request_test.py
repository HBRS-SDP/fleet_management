import json
import logging
import sys
import time
from datetime import timedelta

from mrs.structs.timetable import Timetable
from ropod.pyre_communicator.base_class import RopodPyre
from ropod.utils.timestamp import TimeStamp
from ropod.utils.uuid import generate_uuid
from stn.stp import STP

from fleet_management.config.loader import Config
from fleet_management.db.ccu_store import CCUStore


class TaskRequester(RopodPyre):
    def __init__(self):
        zyre_config = {'node_name': 'task_request_test',
                       'groups': ['ROPOD'],
                       'message_types': ['TASK-REQUEST']}
        super().__init__(zyre_config, acknowledge=False)

        config = Config(initialize=False, log_file='task_request_test')
        store_config = config.config_params.get('ccu_store', dict())
        self.ccu_store = CCUStore(**store_config)
        self.logger = logging.getLogger('task_requester')

        self.robot_ids = config.config_params.get('resources').get('fleet')
        allocation_config = config.config_params.get("plugins").get("task_allocation")
        stp_solver = allocation_config.get('stp_solver')
        self.stp = STP(stp_solver)

    def reset_timetables(self):
        self.logger.info("Resetting timetables")
        for robot_id in self.robot_ids:
            timetable = Timetable(self.stp, robot_id)
            self.ccu_store.update_timetable(timetable)
            self.send_timetable(timetable, robot_id)

    def reset_tasks(self):
        self.logger.info("Resetting tasks")
        tasks_dict = self.ccu_store.get_tasks()
        for task_id, task_dict in tasks_dict.items():
            self.ccu_store.remove_task(task_id)
            self.request_task_delete(task_dict)

    def send_timetable(self, timetable, robot_id):
        logging.debug("Sending timetable to %s", robot_id)
        timetable_msg = dict()
        timetable_msg['header'] = dict()
        timetable_msg['payload'] = dict()
        timetable_msg['header']['type'] = 'TIMETABLE'
        timetable_msg['header']['metamodel'] = 'ropod-msg-schema.json'
        timetable_msg['header']['msgId'] = generate_uuid()
        timetable_msg['header']['timestamp'] = TimeStamp().to_str()

        timetable_msg['payload']['metamodel'] = 'ropod-bid_round-schema.json'
        timetable_msg['payload']['timetable'] = timetable.to_dict()
        self.shout(timetable_msg)

    def request_task_delete(self, task_dict):
        logging.debug("Sending task %s", task_dict['id'])
        task_msg = dict()
        task_msg['header'] = dict()
        task_msg['payload'] = dict()
        task_msg['header']['type'] = 'DELETE-TASK'
        task_msg['header']['metamodel'] = 'ropod-msg-schema.json'
        task_msg['header']['msgId'] = generate_uuid()
        task_msg['header']['timestamp'] = TimeStamp().to_str()

        task_msg['payload']['metamodel'] = 'ropod-bid_round-schema.json'
        task_msg['payload']['task'] = task_dict

        self.shout(task_msg)

    def send_request(self, config_file):
        """ Send task request to fleet management system via pyre

        :config_file: string (path to the config file containing task request
        :returns: None

        """
        self.logger.info("Preparing task request message")
        with open(config_file) as json_file:
            task_request_msg = json.load(json_file)

        task_request_msg['header']['msgId'] = generate_uuid()
        task_request_msg['header']['timestamp'] = TimeStamp().to_str(),

        delta = timedelta(minutes=2)

        task_request_msg['payload']['earliestStartTime'] = TimeStamp(delta)
        self.logger.info("Task earliest start time: %s", task_request_msg['payload']['earliestStartTime'])

        delta = timedelta(minutes=5)

        task_request_msg['payload']['latestStartTime'] = TimeStamp(delta)
        self.logger.info("Task latest start time: %s", task_request_msg['payload']['latestStartTime'])

        self.logger.warning("Sending task request")
        self.shout(task_request_msg)

    def receive_msg_cb(self, msg_content):
        message = self.convert_zyre_msg_to_dict(msg_content)
        if message is None:
            return

        if message['header']['type'] == 'TASK':
            self.logger.debug("Received task message")
            self.terminated = True
        if message['header']['type'] == 'BID':
            self.logger.debug("Received bid message")


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == "invalid":
        config_file = 'config/msgs/task_requests/task-request-mobidik-invalid.json'
    else:
        config_file = 'config/msgs/task_requests/task-request-mobidik.json'

    timeout_duration = 300 # 5 minutes

    test = TaskRequester()
    test.start()

    try:
        time.sleep(20)
        test.reset_timetables()
        test.reset_tasks()
        test.send_request(config_file)
        # TODO: receive msg from ccu for invalid task request instead of timeout
        start_time = time.time()
        while not test.terminated and start_time + timeout_duration > time.time():
            time.sleep(0.5)
    except (KeyboardInterrupt, SystemExit):
        print('Task request test interrupted; exiting')

    print("Exiting test...")
    test.shutdown()
