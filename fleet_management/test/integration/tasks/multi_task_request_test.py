import argparse
import time
import copy
from datetime import timedelta

from fleet_management.test.fixtures.robots import set_initial_positions
from fleet_management.test.fixtures.utils import get_msg_fixture
from fmlib.utils.messages import Message
from fmlib.utils.utils import load_file_from_module, load_yaml
from ropod.pyre_communicator.base_class import RopodPyre
from ropod.utils.timestamp import TimeStamp
from ropod.utils.uuid import generate_uuid


def get_msg(
    msg_template, earliest_pickup_time, latest_pickup_time, pickup_pose, delivery_pose
):
    """Return complete task message for a given task.

    Args:
        msg_template: Dictionary
        earliest_pickup_time: String
        latest_pickup_time: String
        pickup_pose: String
        delivery_pose: String

    Returns:
        task_msg (Dictionary): A copy of the message template with updated fields
    """
    task_msg = copy.deepcopy(msg_template)
    payload = task_msg.payload
    payload["requestId"] = str(generate_uuid())
    payload["pickupLocation"] = pickup_pose
    payload["deliveryLocation"] = delivery_pose

    payload["pickupLocationLevel"] = _get_location_floor(pickup_pose)
    payload["deliveryLocationLevel"] = _get_location_floor(delivery_pose)

    payload["earliestPickupTime"] = earliest_pickup_time
    payload["latestPickupTime"] = latest_pickup_time
    task_msg.update(payload=payload)
    return task_msg


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
    def __init__(self, test_config, msg_template):
        """Sends task request messages to the FMS

        Args:
            test_config: If msg is a dict, a single message will be sent.
                    If msgs is a list, the requester will iterate through them
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
        self.responses_received = 0

    @staticmethod
    def setup(robot_positions):
        set_initial_positions(robot_positions)

    def send_request(self, msg):
        """ Send task request to fleet management system via pyre

        Args:
            msg (dict): A message in ROPOD format
        """
        print("Sending task request")

        self.logger.info("Sending task request")
        self.shout(msg)

    def receive_msg_cb(self, msg_content):
        message = self.convert_zyre_msg_to_dict(msg_content)
        if message is None:
            return

        if message["header"]["type"] == "TASK":
            self.logger.debug(
                "Received dispatch message for task %s" % message["payload"]["taskId"]
            )
            self.responses_received += 1

        if self.responses_received == len(self.test_config.keys()):
            print("Received all dispatch messages from FMS. Terminating...")
            self.terminated = True

    def run_test(self, messages, robot_positions):
        print("Multi-robot multi-task test")
        print("------------------------------------------")
        print("Robot positions: %s" % robot_positions)
        print("Setting up the robots...")
        self.setup(robot_positions)

        # Update the message contents
        print("\nRequests:")
        for m in messages:
            print("\n" + str(m))
            self.send_request(m)
            time.sleep(1)

    def start(self):
        super().start()
        time.sleep(3)

        earliest_pickup_time = TimeStamp(timedelta(minutes=1.5)).to_str()
        latest_pickup_time = TimeStamp(timedelta(minutes=4)).to_str()

        messages = []
        robot_positions = {}
        for case in self.test_config.keys():
            test_data = self.test_config[case]
            # Gather the positions of new robots present in current test case
            for robot, position in test_data["robot_positions"].items():
                if robot not in robot_positions.keys():
                    robot_positions[robot] = position
                elif robot_positions[robot] != position:
                    print("ERROR: Robot positions do not match across all test cases!")

            # Use the same pick and delivery times for all the tasks
            test_data["task"]["earliest_pickup_time"] = earliest_pickup_time
            test_data["task"]["latest_pickup_time"] = latest_pickup_time
            messages.append(get_msg(self.msg_template, **test_data.get("task")))

        self.run_test(messages, robot_positions)


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
    parser.add_argument("-c", "--case-numbers", type=int, nargs="+", default=[9, 10])

    args = parser.parse_args()
    test_cases = load_file_from_module(
        "fleet_management.test.fixtures.msgs.task.requests.brsu",
        args.config + "-test-cases.yaml",
    )
    loaded_test_config = load_yaml(test_cases)

    # Store only the required test cases into test_config
    test_config = {}
    for c in args.case_numbers:
        test_config[c] = loaded_test_config.get(c)

    # Get the message template from a path
    msg_module = args.msg_module
    msg_file = args.msg_file
    msg = Message(**get_msg_fixture(msg_module, msg_file))

    test = TaskRequester(test_config, msg)
    test.start()

    try:
        while not test.terminated:
            time.sleep(0.5)
    except (KeyboardInterrupt, SystemExit):
        print("Task request test interrupted; exiting")

    print("Exiting test...")
    test.shutdown()
