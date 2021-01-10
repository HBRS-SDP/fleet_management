import argparse
import time

from fleet_management.test.fixtures.utils import get_msg_fixture
from fleet_management.test.integration.tasks.task_request_test import TaskRequester
from fmlib.utils.messages import Message
from fmlib.utils.utils import load_file_from_module, load_yaml

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--msg-module", type=str, action="store", default="task.requests.brsu"
    )
    parser.add_argument(
        "--msg-file", type=str, action="store", default="task-request-brsu.json"
    )
    parser.add_argument(
        "--case", type=int, action="store", default=4, help="Test case number"
    )

    args = parser.parse_args()
    case = args.case

    test_cases = load_file_from_module(
        "fleet_management.test.fixtures.msgs.task.requests.brsu", "osm-test-cases.yaml"
    )
    test_config_ = {case: load_yaml(test_cases).get(case)}

    # Get the message template from a path
    msg_module_ = args.msg_module
    msg_file_ = args.msg_file
    msg_ = Message(**get_msg_fixture(msg_module_, msg_file_))

    test = TaskRequester(test_config_, msg_, complete_task=False)
    test.start()

    try:
        while not test.terminated:
            time.sleep(0.5)
    except (KeyboardInterrupt, SystemExit):
        print("Task request test interrupted; exiting")

    print("Task request successful")
