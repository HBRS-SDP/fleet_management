import rospy
from std_msgs.msg import String
from ropod_ros_msgs.msg import Task, TaskProgressDOCK, TaskProgressGOTO, Status

import os
import time

class TaskRelay:
    def __init__(self):
        rospy.init_node('task_relay', anonymous=True)

    def setup(self):
        self.dock_pub = rospy.Publisher('/ropod_com_mediator/ropod_task_feedback/dock', TaskProgressDOCK, queue_size=10)
        self.goto_pub = rospy.Publisher('/ropod_com_mediator/ropod_task_feedback/goto', TaskProgressGOTO, queue_size=10)
        rospy.Subscriber("/ropod_com_mediator/task", Task, self.callback)
        rospy.loginfo("[TASK_RELAY] Setup Complete. Waiting for task message...")

    def callback(self, task):
        task_id = task.task_id
        rospy.loginfo("[TASK_RELAY] Received task_id: {0}".format(task_id))
        actions = task.robot_actions
        rospy.loginfo("[TASK_RELAY] Task contains {0} actions with following action_ids:".format(len(actions)))
        for i in range(len(actions)):
            rospy.loginfo("\tID: {0}, Type: {1}".format(actions[i].action_id, actions[i].type))

        # Choose the first action for responding with a task_progress_msg
        action = actions[0]
        task_progress_msg = self.build_task_progress_message(task_id, action)

        time.sleep(3)

        # Publish a task progress message
        if action.type == "DOCK":
            self.dock_pub.publish(task_progress_msg)
            rospy.loginfo("[TASK_RELAY] Published task progress message on topic {0}".format("/ropod_com_mediator/ropod_task_feedback/dock"))
        else:
            self.goto_pub.publish(task_progress_msg)
            rospy.loginfo("[TASK_RELAY] Published task progress message on topic {0}".format("/ropod_com_mediator/ropod_task_feedback/goto"))

        # Shutdown node
        # self.shutdown()

    def get_status_message(self, domain, module, status_code, sm_state=""):
        status = Status()
        status.domain = domain
        status.module_code = module
        status.status_code = status_code
        #status.sm_state = sm_state

        return status

    def build_task_progress_message(self, task_id, action):
        task_progress_msg = TaskProgressDOCK() if action.type == "DOCK" else TaskProgressGOTO()
        task_progress_msg.task_id = task_id
        task_progress_msg.robot_id = os.environ.get('ROPOD_ID', 'ropod_001')
        task_progress_msg.action_id = action.action_id
        task_progress_msg.action_type = action.type
        task_progress_msg.area_name = "AMK" # Dummy area name
        task_progress_msg.status = self.get_status_message(5, 7, 5) # Dummy status
        task_progress_msg.task_status = self.get_status_message(5, 7, 3) # Dummy task status

        return task_progress_msg

    def shutdown(self):
        rospy.loginfo("[TASK_RELAY] Shutting down node")
        rospy.signal_shutdown("Relay complete")


if __name__ == '__main__':
    task_relay = TaskRelay()
    task_relay.setup()

    rospy.spin()
