#!/usr/bin/env python

import rospy
import threading
from threading import Thread
from std_msgs.msg import String


class Arm_Listener(object):

    def __init__(self, interval=1):
        self.interval = interval

        thread = threading.Thread(target=self.listener, args=())
        thread.daemon = True
        thread.start()

    def callback(self,data):
        rospy.loginfo(rospy.get_caller_id() + data.data)

    def listener(self):
        armTopic = "arm_control"
        print 'ROS Subscribe node subscribed to topic: ' + armTopic
        rospy.init_node('panel_listener', anonymous=True)

        rospy.Subscriber(armTopic, String, self.callback)

        rospy.spin()
