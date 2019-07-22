import logging
import rospy

from rospy_message_converter import message_converter

from ropod_ros_msgs.msg import Task, TaskRequest


class ROSInterface(object):
    """
    ROSInterface.
    """

    def __init__(self, config):
        super(ROSInterface, self).__init__()
        self.logger = logging.getLogger('fms.api.ros')
        rospy.init_node('fms_ros_api', anonymous=False, disable_signals=True)
        self.logger.info("Initialized fleet management ROS interface")

        self.subs = rospy.Subscriber('/fms/task_request', TaskRequest, self.task_cb)
        self.pub = rospy.Publisher('/fms/task/dispatch', Task, queue_size=50)
        rospy.on_shutdown(self.shutdown)

    def task_cb(self, msg):
        msg_dict = message_converter.convert_ros_message_to_dictionary(msg)
        msg_ros = message_converter.convert_dictionary_to_ros_message('ropod_ros_msgs/TaskRequest', msg_dict)
        task = Task()
        self.pub.publish(task)

    def start(self):
        rospy.loginfo("Started ROS interface")

    def shutdown(self):
        rospy.loginfo("Shutting down ROS interface")
        self.logger.warning("Shutting down ROS interface")

    def run(self):
        try:
            self.logger.info('Running')
        except (rospy.ROSInterruptException, KeyboardInterrupt):
            rospy.logerr("Terminating node")


