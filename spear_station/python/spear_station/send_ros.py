#!/usr/bin/env python2

from spear_msgs.msg import drive_cmd

import rospy
import threading
from threading import Thread
from sensor_msgs.msg import Joy


def ros_init():
    global joy_publisher
    global joy_subscriber

    rospy.init_node('drive_control')

    rospy.loginfo("Initializing driver node")
    # Extra configuration needed
    joy_publisher = rospy.Publisher(
        '/drive', drive_cmd, queue_size=50)  # initialize the publisher node
    rospy.loginfo("Started driver node")

    rospy.loginfo("Initializing joy node")
    joy_subscriber = rospy.Subscriber(
        '/joy', Joy, callback)  # initialize the Subscriber node
    rospy.loginfo("Started joy node")


# from Rover1 Code
class SpinROS(threading.Thread):
    def __init__(self):
        super(SpinROS, self).__init__()

    def run(self):
        print('Print thread start')
        rospy.spin()


def callback(data):  # function is called whenever topic is recieved
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


class Driver():
    def __init__(self):
        rospy.init_node("drive_control")
        self.pub = rospy.Publisher("/drive", drive_cmd, queue_size=10)
        rospy.loginfo("Started driver node")

    def send_cmd(self, left, right):
        rospy.loginfo("[Drive] command %d %d" % (left, right))
        msg = drive_cmd()
        msg.left = left
        msg.right = right
        self.pub.publish(msg)

    def is_ros_down(self):
        return rospy.is_shutdown()
