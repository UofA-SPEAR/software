import rospy
from drive_system.msg import drive_cmd


class Driver():
    def __init__(self):
        rospy.init_node("drive_control")
        self.pub = rospy.Publisher("/send_drive", drive_cmd, queue_size=10)
        rospy.loginfo("Started driver node")

    def send_cmd(self, left, right):
        rospy.loginfo("[Drive] command %d %d" % (left, right))
        msg = drive_cmd()
        msg.left = left
        msg.right = right
        self.pub.publish(msg)

    def is_ros_down(self):
        return rospy.is_shutdown()

