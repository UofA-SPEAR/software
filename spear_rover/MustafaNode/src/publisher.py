#!/usr/bin/env python
import rospy
from std_msgs.msg import *

rospy.init_node(MustafaNode)

publisher=rospy.publisher('/joint_states',String,queue_size=1)

rate = rospy.rate(3)

while not rospy.is_shutdown():
    Publisher.publish("Hello")
    rate.sleep()