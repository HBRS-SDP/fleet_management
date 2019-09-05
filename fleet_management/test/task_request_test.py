import json
import logging
import sys
import time
from datetime import timedelta

from ropod.pyre_communicator.base_class import RopodPyre
from ropod.utils.timestamp import TimeStamp
from ropod.utils.uuid import generate_uuid

from fleet_management.db.ccu_store import CCUStore


class TaskRequester(RopodPyre):
    def __init__(self):
        zyre_config = {'node_name': 'task_request_test',
                       'groups': ['ROPOD'],
                       'message_types': ['TASK-REQUEST']}
        super().__init__(zyre_config, acknowledge=False)

        self.logger = logging.getLogger('task_requester')

        self.ccu_store = CCUStore('ropod_ccu_store')
        robot_id = 'ropod_001'

        self.robot_store = CCUStore('ropod_store_' + robot_id)

    def tear_down(self):
        self.logger.info("Resetting the ccu_store")
        self.ccu_store.clean()
        self.logger.info("Resetting the robot_store")
        self.robot_store.clean()

    def send_request(self, msg_file):
        """ Send task request to fleet management system via pyre

        :config_file: string (path to the config file containing task request
        :returns: None

        """
        self.logger.info("Preparing task request message")
        with open(msg_file) as json_file:
            task_request_msg = json.load(json_file)

        task_request_msg['header']['msgId'] = generate_uuid()
        task_request_msg['header']['timestamp'] = TimeStamp().to_str()

        delta = timedelta(minutes=2)

        task_request_msg['payload']['earliestPickupTime'] = TimeStamp(delta).to_str()
        self.logger.info("Task earliest pickup time: %s", task_request_msg['payload']['earliestPickupTime'])

        delta = timedelta(minutes=5)

        task_request_msg['payload']['latestPickupTime'] = TimeStamp(delta).to_str()
        self.logger.info("Task latest pickup time: %s", task_request_msg['payload']['latestPickupTime'])

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

    timeout_duration = 300  # 5 minutes

    test = TaskRequester()
    test.start()

    try:
        time.sleep(20)
        test.tear_down()
        time.sleep(5)
        test.send_request(config_file)
        # TODO: receive msg from ccu for invalid task request instead of timeout
        start_time = time.time()
        while not test.terminated and start_time + timeout_duration > time.time():
            time.sleep(0.5)
        test.tear_down()
    except (KeyboardInterrupt, SystemExit):
        print('Task request test interrupted; exiting')

    print("Exiting test...")
    test.shutdown()
