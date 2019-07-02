import time
import json
import sys
from datetime import timedelta

from ropod.pyre_communicator.base_class import RopodPyre
from ropod.utils.uuid import generate_uuid
from ropod.utils.timestamp import TimeStamp as ts
from fleet_management.config.loader import Config
from fleet_management.db.ccu_store import CCUStore


class TaskRequester(RopodPyre):
    def __init__(self):
        zyre_config = {'node_name': 'task_request_test',
                       'groups': ['ROPOD'],
                       'message_types': ['TASK-REQUEST']}
        super().__init__(zyre_config, acknowledge=False)

        config = Config('../../config/fms_config-v2.yaml', False)
        store_config = config.config_params.get('ccu_store', dict())
        self.ccu_store = CCUStore(**store_config)

        self.robots = config.config_params.get('resources').get('fleet')

    def reset_robots_schedule(self):
        self.logger.info("Cleaning robots' schedules")

        for robot in self.robots:
            robot_schedule = self.ccu_store.get_robot_schedule(robot)

            if robot_schedule:
                for task in robot_schedule:
                    self.ccu_store.remove_task_from_robot_schedule(robot, task.id)
                self.send_reset_schedule_msg(robot)

    def send_reset_schedule_msg(self, robot_id):
        reset_schedule_msg = dict()
        reset_schedule_msg["header"] = dict()
        reset_schedule_msg["payload"] = dict()

        reset_schedule_msg["header"]["type"] = "RESET-SCHEDULE"
        reset_schedule_msg["header"]["metamodel"] = "ropod-msg-schema.json"
        reset_schedule_msg["header"]["msgId"] = generate_uuid()
        reset_schedule_msg["header"]["timestamp"] = ts.get_time_stamp()

        reset_schedule_msg["payload"]["metamodel"] = "ropod-msg-schema.json"

        self.logger.info("Sending RESET-SCHEDULE msg to %s", robot_id)

        self.whisper(reset_schedule_msg, peer=robot_id + '_proxy')

    def send_request(self, config_file):
        """ Send task request to fleet management system via pyre

        :config_file: string (path to the config file containing task request
        :returns: None

        """
        self.logger.info("Preparing task request message")
        with open(config_file) as json_file:
            task_request_msg = json.load(json_file)

        task_request_msg['header']['msgId'] = generate_uuid()
        task_request_msg['header']['timestamp'] = ts.get_time_stamp()

        delta = timedelta(minutes=1)

        task_request_msg['payload']['earliestStartTime'] = ts.get_time_stamp(delta)

        delta = timedelta(minutes=1, seconds=30)

        task_request_msg['payload']['latestStartTime'] = ts.get_time_stamp(delta)

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
        time.sleep(10)
        test.reset_robots_schedule()
        test.send_request(config_file)
        # TODO: receive msg from ccu for invalid task request instead of timeout
        start_time = time.time()
        while not test.terminated and start_time + timeout_duration > time.time():
            time.sleep(0.5)
    except (KeyboardInterrupt, SystemExit):
        print('Task request test interrupted; exiting')

    print("Exiting test...")
    test.shutdown()
