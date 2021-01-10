import argparse
import time
from datetime import timedelta

from fleet_management.test.fixtures.robots import set_initial_positions
from fleet_management.test.fixtures.utils import get_msg_fixture
from fmlib.utils.messages import Message
from fmlib.utils.utils import load_file_from_module, load_yaml
from ropod.pyre_communicator.base_class import RopodPyre
from ropod.utils.timestamp import TimeStamp
from ropod.utils.uuid import generate_uuid
from ropod.structs.status import TaskStatus


def update_msg_fields(msg, pickup_pose, delivery_pose):
    msg.refresh()
    payload = msg.payload
    payload["requestId"] = str(generate_uuid())
    payload["pickupLocation"] = pickup_pose
    payload["deliveryLocation"] = delivery_pose

    payload["pickupLocationLevel"] = _get_location_floor(pickup_pose)
    payload["deliveryLocationLevel"] = _get_location_floor(delivery_pose)

    delta = timedelta(minutes=1)
    payload["earliestPickupTime"] = TimeStamp(delta).to_str()
    delta = timedelta(minutes=4)
    payload["latestPickupTime"] = TimeStamp(delta).to_str()
    msg.update(payload=payload)


def _get_location_floor(location):
    """Return the floor number of a given location.
    For ROPOD, this can either be done through the OSM path planner or
    by parsing an Area string

    Args:
        location: An Area string

    Returns:
        floor (int): The floor number of an area
    """
    return int(location.split("_")[2].replace("L", ""))


class TaskRequester(RopodPyre):
    def __init__(self, test_config, msg_template, complete_task=True):
        """Sends task request messages to the FMS

        Args:
            test_config: If msg is a dict, a single message will be sent.
                    If msgs is a list, the requester will iterate through them

            complete_task: If true, a task-request msg with task-status COMPLETED is sent upon
                            reception of a task msg
        """
        zyre_config = {
            "node_name": "task_request_test",
            "groups": ["ROPOD"],
            "message_types": ["TASK-REQUEST"],
        }
        super().__init__(zyre_config, acknowledge=False)

        if isinstance(test_config, dict):
            self.test_config = test_config
        else:
            print("Unrecognized test configuration. Test won't work")
            self.test_config = dict()

        self.msg_template = msg_template
        self.complete_task = complete_task

    @staticmethod
    def setup(robot_positions):
        set_initial_positions(robot_positions)

    def send_msg(self, msg):
        """ Send msg to fleet management system via pyre

        Args:
            msg (dict): A message in ROPOD format
        """
        self.logger.info("Sending %s msg", msg["header"]["type"])
        self.shout(msg)

    def receive_msg_cb(self, msg_content):
        message = self.convert_zyre_msg_to_dict(msg_content)
        if message is None:
            return

        if message["header"]["type"] == "TASK":
            task_id = message["payload"]["taskId"]
            self.logger.debug("Received dispatch message for task %s" % task_id)
            if self.test_config:
                test_case_ = self.test_config.popitem()
                self.run_test(test_case_[1])
            else:
                self.terminate(task_id)
        elif message["header"]["type"] == "INVALID-TASK-REQUEST":
            self.logger.debug(
                "Received reply for invalid task %s" % message["payload"]["requestId"]
            )
            self.terminated = True

    def run_test(self, test_case):
        robot_positions_ = test_case.pop("robot_positions")
        print(
            test_case.pop("description")
            + "\n------------------------------------------"
        )
        print("Robot positions: %s" % robot_positions_)
        time.sleep(5)
        self.setup(robot_positions_)

        # Update the message contents
        update_msg_fields(self.msg_template, **test_case.get("task"))
        print("Request:")
        print(self.msg_template)

        time.sleep(5)
        self.send_msg(self.msg_template)

    def start(self):
        super().start()
        time.sleep(3)
        if len(self.test_config) > 1:
            print("Running %i test cases" % len(self.test_config))
        test_case_ = self.test_config.popitem()[1]
        self.run_test(test_case_)

    def terminate(self, task_id):
        if self.complete_task:
            self.send_complete_task(task_id)
        self.terminated = True

    def send_complete_task(self, task_id):
        """ Sends task-status msg with status COMPLETED

        Args:
            task_id: id of the task to complete

        """
        msg = Message(**get_msg_fixture("task.progress", "task-status.json"))
        msg.refresh()
        payload = msg.payload
        payload["taskId"] = str(task_id)
        payload["taskStatus"] = TaskStatus.COMPLETED
        print("Task status:")
        print(msg)
        time.sleep(5)
        self.send_msg(msg)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, action="store", default="osm")
    parser.add_argument(
        "--msg-module", type=str, action="store", default="task.requests.brsu"
    )
    parser.add_argument(
        "--msg-file", type=str, action="store", default="task-request-brsu.json"
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--case", type=int, action="store", default=4, help="Test case number"
    )
    group.add_argument("--all", action="store_true")

    args = parser.parse_args()
    case = args.case
    config = args.config

    test_cases = load_file_from_module(
        "fleet_management.test.fixtures.msgs.task.requests.brsu",
        config + "-test-cases.yaml",
    )

    if args.all:
        test_config_ = load_yaml(test_cases)
    else:
        test_config_ = {case: load_yaml(test_cases).get(case)}

    # Get the message template from a path
    msg_module_ = args.msg_module
    msg_file_ = args.msg_file
    msg_ = Message(**get_msg_fixture(msg_module_, msg_file_))

    test = TaskRequester(test_config_, msg_)
    test.start()

    try:
        while not test.terminated:
            time.sleep(0.5)
    except (KeyboardInterrupt, SystemExit):
        print("Task request test interrupted; exiting")

    print("Exiting test...")
    test.shutdown()
