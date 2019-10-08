import time

from ropod.pyre_communicator.base_class import RopodPyre

from fleet_management.test.fixtures.utils import get_msg_fixture


class MessageShouter(RopodPyre):
    def __init__(self, groups):
        zyre_config = {
            'node_name': 'shouter',
            'groups': groups,
            'message_types': []
        }
        super().__init__(zyre_config)
        self.last_msg = None

    def send_msg_from_file(self, msg_path, msg_file, groups=None, new_values=None):
        print("Sending msg: %s" % msg_file)

        msg = get_msg_fixture(msg_path, msg_file)

        return self.send_msg(msg, groups, new_values)

    def send_msg(self, msg, groups=None, new_values=None):

        if new_values is not None:
            print("Updating new values")
            msg.update(new_values)

        if groups is None:
            print("Shouting to all groups")
            self.shout(msg)
        else:
            print("Shouting to %s group" % groups)
            self.shout(msg, groups=groups)

        return msg

    def receive_msg_cb(self, msg_content):
        msg = self.convert_zyre_msg_to_dict(msg_content)
        if msg is None:
            return

        self.last_msg = msg


if __name__ == '__main__':
    import argparse
    shouter = MessageShouter(['ROPOD'])
    shouter.start()

    parser = argparse.ArgumentParser()
    parser.add_argument('module', type=str, action='store', help='Module of the message to send')
    parser.add_argument('file', type=str, action='store', help='Full name of the file to use')
    args = parser.parse_args()
    msg_file_ = args.file
    msg_module_ = args.module

    try:
        time.sleep(2)
        shouter.send_msg_from_file(msg_module_, msg_file_)
        while not shouter.terminated:
            time.sleep(0.5)
        raise KeyboardInterrupt
    except (KeyboardInterrupt, SystemExit):
        print("Exiting test...")
        shouter.shutdown()
