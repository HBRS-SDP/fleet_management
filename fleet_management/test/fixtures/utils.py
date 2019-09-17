import json

from importlib_resources import open_text


def get_msg_fixture(msg_module, msg_file):
    full_path = 'fleet_management.test.fixtures.msgs.' + msg_module

    with open_text(full_path, msg_file) as json_msg:
        msg = json.load(json_msg)

    return msg

