from __future__ import print_function
import time
import json
import os.path
from datetime import timedelta

from ropod.pyre_communicator.base_class import RopodPyre
from ropod.utils.uuid import generate_uuid
from ropod.utils.timestamp import TimeStamp as ts
from fleet_management.config.config_file_reader import ConfigFileReader
from fleet_management.db.ccu_store import CCUStore


class TaskRequester(RopodPyre):
    def __init__(self):
        super().__init__('task_request_test', ['ROPOD'], ['TASK-REQUEST'], verbose=True, acknowledge=True)

    @staticmethod
    def clean_robots_schedule():

        config_file = "../../config/ccu_config.yaml"
        config_params = ConfigFileReader.load(config_file)

        ccu_store = CCUStore(config_params.ccu_store_db_name)
        robots_params = config_params.ropods

        for robot_params in robots_params:
            robot_schedule = ccu_store.get_robot_schedule(robot_params.id)
            print("Robot schedule before: ", robot_schedule)
            for task in robot_schedule:
                ccu_store.remove_task_from_robot_schedule(robot_params.id, task.id)

            robot_schedule = ccu_store.get_robot_schedule(robot_params.id)
            print("Robot schedule after:", robot_schedule)

    def send_request(self):
        print("Preparing task request message")
        with open('config/msgs/task_requests/task-request-mobidik.json') as json_file:
            task_request_msg = json.load(json_file)

        task_request_msg['header']['msgId'] = generate_uuid()
        task_request_msg['header']['timestamp'] = ts.get_time_stamp()

        earliest_start_time = timedelta(minutes=1)

        task_request_msg['payload']['earliestStartTime'] = ts.get_time_stamp(earliest_start_time)

        latest_start_time = timedelta(minutes=1, seconds=30)

        task_request_msg['payload']['latestStartTime'] = ts.get_time_stamp(latest_start_time)

        print("Sending task request")
        self.shout(task_request_msg)

    def receive_msg_cb(self, msg_content):
        message = self.convert_zyre_msg_to_dict(msg_content)
        if message is None:
            return

        if message['header']['type'] == 'TASK':
            print("Received task message")
            self.terminated = True


if __name__ == '__main__':
    test = TaskRequester()
    test.start()

    try:
        time.sleep(10)
        TaskRequester.clean_robots_schedule()
        test.send_request()
        time.sleep(10)
        test.send_request()
        while not test.terminated:
            time.sleep(0.5)
        raise KeyboardInterrupt
    except (KeyboardInterrupt, SystemExit):
        print("Exiting test...")
        test.shutdown()
        print('Task request test interrupted; exiting')
