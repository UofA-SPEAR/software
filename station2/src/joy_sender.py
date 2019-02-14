#!/usr/bin/env python
import rospy

# Import message types
from sensor_msgs.msg import Joy
from drive_controls.msg import drive_cmd

# 0 indexed array
left_axis = 1
right_axis = 4
pub = rospy.Publisher('/drive', drive_cmd, queue_size=10)

def joy_callback(data):
    rospy.loginfo("Joystick Input: [L, R] [ %f, %f ]",
            data.axes[left_axis], data.axes[right_axis])

    msg = drive_cmd()
    msg.left = data.axes[left_axis] * 100
    msg.right = data.axes[right_axis] * 100
    pub.publish(msg)


# Not sure what I should call this, not too used to python
def joy_sender():
    rospy.init_node('joy_publisher')
    rospy.Subscriber('/joy', Joy, joy_callback);

    rospy.loginfo("Joystick sender started!")

    rospy.spin()


if __name__ == '__main__':
    joy_sender()
    
