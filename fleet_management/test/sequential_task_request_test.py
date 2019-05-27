from __future__ import print_function
import time
import json
from datetime import timedelta

from ropod.pyre_communicator.base_class import RopodPyre
from ropod.utils.uuid import generate_uuid
from ropod.structs.task import Task
from ropod.structs.status import COMPLETED
from ropod.utils.timestamp import TimeStamp as ts
from fleet_management.config.config_file_reader import ConfigFileReader
from fleet_management.db.ccu_store import CCUStore

""" Test for sending multiple task requests for tasks with overlapping time windows
"""


class TaskRequester(RopodPyre):
    def __init__(self, n_task_requests):
        super().__init__("task_request_test", ["ROPOD"], ["TASK-REQUEST"], verbose=True, acknowledge=True)
        self.n_task_requests = n_task_requests
        self.n_sent_requests = 0
        self.completed_tasks = dict()
        self.task_alternative_timeslots = dict()
        self.task_request_msg = dict()

    def clean_robots_schedule(self):
        print("Cleaning robots' schedules")

        config_file = "../../config/ccu_config.yaml"
        config_params = ConfigFileReader.load(config_file)

        ccu_store = CCUStore(config_params.ccu_store_db_name)
        robots_params = config_params.ropods

        for robot_params in robots_params:
            robot_schedule = ccu_store.get_robot_schedule(robot_params.id)
            print("Robot " + robot_params.id + " schedule before: ", robot_schedule)
            if robot_schedule:
                for task in robot_schedule:
                    ccu_store.remove_task_from_robot_schedule(robot_params.id, task.id)
                self.send_reset_schedule_msg(robot_params.id)

            robot_schedule = ccu_store.get_robot_schedule(robot_params.id)
            print("Robot " + robot_params.id + " schedule after: ", robot_schedule)

    def send_reset_schedule_msg(self, robot_id):
        reset_schedule_msg = dict()
        reset_schedule_msg["header"] = dict()
        reset_schedule_msg["payload"] = dict()

        reset_schedule_msg["header"]["type"] = "RESET-SCHEDULE"
        reset_schedule_msg["header"]["metamodel"] = "ropod-msg-schema.json"
        reset_schedule_msg["header"]["msgId"] = generate_uuid()
        reset_schedule_msg["header"]["timestamp"] = ts.get_time_stamp()

        reset_schedule_msg["payload"]["metamodel"] = "ropod-msg-schema.json"

        print("Sending RESET-SCHEDULE msg to:", robot_id)

        self.whisper(reset_schedule_msg, peer=robot_id)

    def send_request(self):
        self.n_sent_requests += 1

        if not self.task_request_msg:

            with open("config/msgs/task_requests/task-request-mobidik.json") as json_file:
                self.task_request_msg = json.load(json_file)

            earliest_start_time = timedelta(minutes=1)

            self.task_request_msg["payload"]["earliestStartTime"] = ts.get_time_stamp(earliest_start_time)

            latest_start_time = timedelta(minutes=1, seconds=30)

            self.task_request_msg["payload"]["latestStartTime"] = ts.get_time_stamp(latest_start_time)

        self.task_request_msg["header"]["msgId"] = generate_uuid()
        self.task_request_msg["header"]["timestamp"] = ts.get_time_stamp()

        print("est:", self.task_request_msg["payload"]["earliestStartTime"])
        print("lst:", self.task_request_msg["payload"]["latestStartTime"])

        print("Sending task request")
        self.shout(self.task_request_msg)

    def complete_task(self, task_id):
        task_progress_msg = dict()
        task_progress_msg["header"] = dict()
        task_progress_msg["payload"] = dict()

        task_progress_msg["header"]["type"] = "TASK-PROGRESS"
        task_progress_msg["header"]["metamodel"] = "ropod-msg-schema.json"
        task_progress_msg["header"]["msgId"] = generate_uuid()
        task_progress_msg["header"]["timestamp"] = ts.get_time_stamp()

        task_progress_msg["payload"]["taskId"] = task_id
        task_progress_msg["payload"]["robotId"] = "ropod_001"
        task_progress_msg["payload"]["actionId"] = ""
        task_progress_msg["payload"]["actionType"] = ""
        task_progress_msg["payload"]["status"] = dict()
        task_progress_msg["payload"]["status"]["areaName"] = ""
        task_progress_msg["payload"]["status"]["actionStatus"] = ""
        task_progress_msg["payload"]["status"]["sequenceNumber"] = ""
        task_progress_msg["payload"]["status"]["totalNumber"] = ""
        task_progress_msg["payload"]["status"]["taskStatus"] = COMPLETED

        task_progress_msg["payload"]["metamodel"] = "ropod-msg-schema.json"

        print("Sending TASK-PROGRESS msg: COMPLETED")

        self.shout(task_progress_msg)

    def check_terminate_test(self):
        if len(self.completed_tasks) + len(self.task_alternative_timeslots) == self.n_task_requests:
            print("Terminating test ...")
            self.clean_robots_schedule()
            self.terminated = True

    def check_send_request(self):
        if self.n_sent_requests < self.n_task_requests:
            self.send_request()

    def receive_msg_cb(self, msg_content):
        message = self.convert_zyre_msg_to_dict(msg_content)
        if message is None:
            return

        if message["header"]["type"] == "TASK":
            task = Task.from_dict(message["payload"])

            # Complete task_id after receiving the first TASK msg for the task_id
            if task.id not in self.completed_tasks.keys():
                print("Received task message for task", task.id)
                self.complete_task(task.id)
                self.completed_tasks[task.id] = task

            self.check_send_request()
            self.check_terminate_test()

        elif message["header"]["type"] == 'TASK-ALTERNATIVE-TIMESLOT':
            task_alternative_timeslot = dict()
            task_id = message['payload']['task_id']
            task_alternative_timeslot['robot_id'] = message['payload']['robot_id']
            task_alternative_timeslot['start_time'] = message['payload']['start_time']
            print("Received alternative timeslot : ", task_alternative_timeslot, task_id)
            self.task_alternative_timeslots[task_id] = task_alternative_timeslot
            self.check_send_request()
            self.check_terminate_test()


if __name__ == "__main__":
    test = TaskRequester(2)
    test.start()

    try:
        time.sleep(10)
        test.clean_robots_schedule()
        test.send_request()
        while not test.terminated:
            time.sleep(0.5)
        raise KeyboardInterrupt
    except (KeyboardInterrupt, SystemExit):
        print("Exiting test...")
        test.shutdown()
        print("Task request test interrupted; exiting")
