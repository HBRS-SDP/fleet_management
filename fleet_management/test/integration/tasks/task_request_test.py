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


def update_msg_fields(msg, pickup_pose, delivery_pose):
    msg.refresh()
    payload = msg.payload
    payload['requestId'] = str(generate_uuid())
    payload['pickupLocation'] = pickup_pose
    payload['deliveryLocation'] = delivery_pose

    payload['pickupLocationLevel'] = _get_location_floor(pickup_pose)
    payload['deliveryLocationLevel'] = _get_location_floor(delivery_pose)

    delta = timedelta(minutes=1)
    payload['earliestPickupTime'] = TimeStamp(delta).to_str()
    delta = timedelta(minutes=5)
    payload['latestPickupTime'] = TimeStamp(delta).to_str()
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
    return int(location.split('_')[2].replace('L', ''))


class TaskRequester(RopodPyre):
    def __init__(self):
        zyre_config = {'node_name': 'task_request_test',
                       'groups': ['ROPOD'],
                       'message_types': ['TASK-REQUEST']}
        super().__init__(zyre_config, acknowledge=False)

    @staticmethod
    def setup(robot_positions):
        set_initial_positions(robot_positions)

    def send_request(self, msg):
        """ Send task request to fleet management system via pyre

        Args:
            msg (dict): A message in ROPOD format
        """
        # self.logger.info("Preparing task request message")
        # task_request_msg = get_msg_fixture(msg_module, msg_file)
        #
        # task_request_msg['header']['msgId'] = generate_uuid()
        # task_request_msg['header']['timestamp'] = TimeStamp().to_str()
        #
        # delta = timedelta(minutes=2)
        #
        # task_request_msg['payload']['earliestPickupTime'] = TimeStamp(delta).to_str()
        # self.logger.info("Task earliest pickup time: %s", task_request_msg['payload']['earliestPickupTime'])
        #
        # delta = timedelta(minutes=5)
        #
        # task_request_msg['payload']['latestPickupTime'] = TimeStamp(delta).to_str()
        # self.logger.info("Task latest pickup time: %s", task_request_msg['payload']['latestPickupTime'])
        #
        # self.logger.warning("Sending task request")
        self.shout(msg)

    def receive_msg_cb(self, msg_content):
        message = self.convert_zyre_msg_to_dict(msg_content)
        if message is None:
            return

        if message['header']['type'] == 'TASK':
            self.logger.debug("Received task message")
            self.terminated = True


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--case', type=int, action='store', default=4, help='Test case number')
    parser.add_argument('--msg-module', type=str, action='store', default='task.requests.brsu')
    parser.add_argument('--msg-file', type=str, action='store', default='task-request-brsu.json')

    args = parser.parse_args()
    case = args.case

    test = load_file_from_module('fleet_management.test.fixtures.msgs.task.requests.brsu', 'test-cases.yaml')
    test_config = load_yaml(test).get(case)

    robot_positions_ = test_config.pop('robot_positions')
    print(test_config.pop('description')+"\n------------------------------------------")
    print("Robot positions: %s" % robot_positions_)

    # Get the message from a path
    msg_module = args.msg_module
    msg_file = args.msg_file
    msg_ = Message(**get_msg_fixture(msg_module, msg_file))

    # Update the message contents
    update_msg_fields(msg_, **test_config.get('task'))
    print("Request:")
    print(msg_)

    test = TaskRequester()
    test.start()

    try:
        time.sleep(20)
        test.setup(robot_positions_)
        time.sleep(5)
        test.send_request(msg_)
        while not test.terminated:
            time.sleep(0.5)
    except (KeyboardInterrupt, SystemExit):
        print('Task request test interrupted; exiting')

    print("Exiting test...")
    test.shutdown()
