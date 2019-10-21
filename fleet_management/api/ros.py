import rospy
from ropod_ros_msgs.msg import Task
from rospy_message_converter import message_converter
from fmlib.api.ros import ROSInterface as ROSInterfaceBase


class ROSInterface(ROSInterfaceBase):
    """
    ROSInterface.
    """

    def task_cb(self, msg):
        msg_dict = message_converter.convert_ros_message_to_dictionary(msg)
        msg_ros = message_converter.convert_dictionary_to_ros_message('ropod_ros_msgs/TaskRequest', msg_dict)
        task = Task()
        self._publisher_dict['/fms/task'].publish(task)

    def start(self):
        rospy.loginfo("Started ROS interface of rospy")
        self.logger.error("Started ROS interface!")

    def shutdown(self):
        rospy.loginfo("Shutting down rospy")
        self.logger.warning("Shutting down ROS interface")

    def run(self):
        if not rospy.is_shutdown():
            try:
                #self.logger.info('Running')
                pass
            except (rospy.ROSInterruptException, KeyboardInterrupt):
                rospy.logerr("Terminating node")
                self.logger.error('Terminating ROS interface')


