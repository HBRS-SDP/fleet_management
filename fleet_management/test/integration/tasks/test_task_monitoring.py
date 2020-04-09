import time

from fleet_management.test.fixtures.message_sender import MessageShouter
from fleet_management.test.fixtures.utils import get_msg_fixture
from ropod.structs.status import TaskStatus, ActionStatus

if __name__ == '__main__':
    msg_sender = MessageShouter(['ROPOD'])
    msg_sender.start()
    time.sleep(2)
    msg = get_msg_fixture('task.progress', 'task-progress.json')
    print(msg)
    # NOTE: The ID of the task and action must be matched to an existing task in the ccu store
    msg['payload']['taskId'] = 'fac407ff-d379-4265-8508-31d150d08aff'
    msg['payload']['taskStatus'] = TaskStatus.FAILED
    msg['payload']['taskProgress']['actionId'] = "492f0396-4fd8-4ef2-b9c5-212b3fbd503e"
    msg['payload']['taskProgress']['status'] = ActionStatus.FAILED
    msg_sender.shout(msg)
    msg_sender.shutdown()
