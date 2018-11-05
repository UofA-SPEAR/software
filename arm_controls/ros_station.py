#!/usr/bin/env python

import rospy
import threading
from threading import Thread
from std_msgs.msg import String


''' Lyndon's old garbage code
def __init__(self, interval=1):
    self.interval = interval

    thread = threading.Thread(target=self.listener, args=())
    thread.daemon = True
    thread.start()

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + data.data)

def listener():
    armTopic = "arm_control"
    print 'ROS Subscribe node subscribed to topic: ' + armTopic # check to see that thread is running
    rospy.init_node('panel_node', anonymous=True)

    rospy.Subscriber(armTopic, String, callback)

    # rospy.spin()
'''

# from Rover1 Code
class SpinROS(threading.Thread):
    def __init__(self):
        super(SpinROS, self).__init__()

    def run(self):
        rospy.spin()

def ros_init():
    global arm_publisher

    rospy.init_node('arm_controller')
    rospy.loginfo("Initializing the arm input node")
    
    # Extra configuration needed
    # arm_publisher = rospy.Publisher('/user_arm_controls', input_arm, queue_size=50)
