import time

from fleet_management.test.fixtures.message_sender import MessageShouter
from fleet_management.test.fixtures.utils import get_msg_fixture
from fmlib.utils.messages import Header


def set_initial_positions(positions):
    """Sends a JSON message to set the initial position of the robots

    Args:
        positions (dict): A dictionary with keys as robot IDs
                        and values as the robot position

    """
    msg_sender = MessageShouter(['ROPOD'])
    msg_sender.start()
    msg = get_msg_fixture('robot', 'robot-position.json')
    time.sleep(1)
    for robot, position in positions.items():
        msg['header'] = Header("ROBOT-POSE", meta_model="ropod-msg-schema.json")
        msg['payload']['robotId'] = robot
        msg['payload']['subarea'] = position
        msg_sender.send_msg(msg, groups=['ROPOD'])
    time.sleep(1)

    msg_sender.shutdown()


if __name__ == '__main__':
    robot_positions = {'ropod_001': 'AMK_D_L-1_C39',
                       'ropod_002': 'AMK_D_L-1_C39'}
    set_initial_positions(robot_positions)
