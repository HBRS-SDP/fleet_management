import json

from importlib_resources import open_text


def get_msg_from_path(msg_path):
    msg_subpath = msg_path.split(sep='/')
    full_path = 'fleet_management.test.fixtures.msgs.' + msg_subpath[0]

    with open_text(full_path, msg_subpath[1]) as json_msg:
        msg = json.load(json_msg)

    return msg

