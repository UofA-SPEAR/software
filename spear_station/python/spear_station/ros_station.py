#!/usr/bin/env python

import rospy
import threading
from threading import Thread
# input_axes, input_buttons # Messages need to be compiled
from arm_controls.msg import arm_position
from sensor_msgs.msg import Joy


def ros_init():
    global joy_publisher
    global joy_subscriber

    rospy.init_node('arm_controller')
    rospy.loginfo("Initializing the arm input node")

    # Extra configuration needed
    joy_publisher = rospy.Publisher(
        '/user_arm_controls', arm_position,
        queue_size=50)  # initialize the publisher node
    joy_subscriber = rospy.Subscriber(
        '/joy', Joy, callback)  # initialize the Subscriber node


# from Rover1 Code


class SpinROS(threading.Thread):
    def __init__(self):
        super(SpinROS, self).__init__()

    def run(self):
        print('Print thread start')
        rospy.spin()


def callback(data):  # function is called whenever topic is recieved
    # rospy.loginfo('Nice, a topic has been recieved')
    rospy.loginfo(data.axes)
    [
        joyData.l_stick_x, joyData.l_stick_y, joyData.l_bumper,
        joyData.r_stick_x, joyData.r_stick_y, joyData.r_bumper
    ] = data.axes
    joyData.dpad = [
        data.buttons[13], data.buttons[14], data.buttons[15], data.buttons[16]
    ]  # up down left right


class joyData:
    l_stick_x = 0
    l_stick_y = 0
    r_stick_x = 0
    r_stick_y = 0
    l_bumper = 0
    r_bumper = 0
    dpad = [0, 0, 0, 0]

    def __init__(self):
        pass


def publish(axes):
    joy_publisher.publish(axes[0], axes[1], axes[2], axes[3], axes[4], axes[5])
