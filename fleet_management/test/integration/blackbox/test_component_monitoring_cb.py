from fleet_management.test.fixtures.message_sender import MessageShouter
import time


if __name__ == '__main__':
    msg_sender = MessageShouter(['ROPOD'])
    msg_sender.start()
    time.sleep(2)
    msg_sender.send_msg_from_file('robot', 'component-monitoring-status.json')
    msg_sender.shutdown()
