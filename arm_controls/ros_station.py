#!/usr/bin/env python

import rospy
import threading
from threading import Thread
from arm_controls.msg import arm_output # input_axes, input_buttons # Messages need to be compiled
from sensor_msgs.msg import Joy

# from Rover1 Code
class SpinROS(threading.Thread):
    def __init__(self):
        super(SpinROS, self).__init__()

    def run(self):
        print('Print thread start')
        rospy.spin()

def callback(data): # function is called whenever topic is recieved
    rospy.loginfo('Nice, a topic has been recieved')

def ros_init():
    global joy_publisher
    global button_publisher

    rospy.init_node('arm_controller')
    rospy.loginfo("Initializing the arm input node")
    
    # Extra configuration needed
    joy_publisher = rospy.Publisher('/user_arm_controls', arm_output, queue_size=50) # initialize the publisher node
    joy_subscriber = rospy.Subscriber('/user_arm_controls', arm_output, callback) # initialize the Subscriber node
