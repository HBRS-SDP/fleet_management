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

from task_request_test import update_msg_fields, _get_location_floor, TaskRequester

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--msg-module', type=str, action='store', default='task.requests.brsu')
    parser.add_argument('--msg-file', type=str, action='store', default='task-request-brsu.json')
    parser.add_argument('--case', type=int, action='store', default=4, help='Test case number')

    args = parser.parse_args()
    case = args.case

    test_cases = load_file_from_module('fleet_management.test.fixtures.msgs.task.requests.brsu', 'test-cases.yaml')
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
        print('Task request test interrupted; exiting')

    print("Task request successful")
