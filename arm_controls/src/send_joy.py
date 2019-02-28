#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy


class Arm_Controller():
    def __init__(self):
        rospy.init_node("arm_control", anonymous=True)
        self.pub = rospy.Publisher('joy', Joy, queue_size=10)
        rospy.loginfo("Started arm_control node")
        rate = rospy.Rate(60)

    def send_joystate(self, axes, buttons):
        rospy.loginfo("[Joystick] command %f %d" % (axes, buttons))
        msg = Joy()
        msg.axes = axes
        msg.buttons = buttons
        self.pub.publish(msg)

    def is_ros_down(self):
        return rospy.is_shutdown()
# axes = 0
# buttons = 0
# def talker():
#     rospy.init_node("arm_control", anonymous=True)
#     pub = rospy.Publisher('joy', Joy, queue_size=10)
#     rospy.loginfo("Started arm_control node")
#     rate = rospy.Rate(60)

#     while not rospy.is_shutdown():
#         rospy.loginfo("[Joystick] command %d %d" % (axes, buttons))
#         msg = Joy()
#         msg.axes = axes
#         msg.buttons = buttons
#         pub.publish(msg)
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         talker()
#     except rospy.ROSInterruptException:
#         pass