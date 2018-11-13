#!/usr/bin/env python

import rospy
import threading
from threading import Thread
#from joyinput.msg import input_axes, input_buttons # Messages need to be compiled

# from Rover1 Code
class SpinROS(threading.Thread):
    def __init__(self):
        super(SpinROS, self).__init__()

    def run(self):
        rospy.spin()

def ros_init():
    global joy_publisher
    global button_publisher

    rospy.init_node('arm_controller')
    rospy.loginfo("Initializing the arm input node")
    
    # Extra configuration needed
    #joy_publisher = rospy.Publisher('/user_arm_controls', input_axes, queue_size=50)
    #button_publisher = rospy.Publisher('/user_arm_controls', input_buttons, queue_size=50)
