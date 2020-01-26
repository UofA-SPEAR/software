#! /usr/bin/env python
import rospy
import random
import math
from std_msgs.msg import *
def talker():
    position = Float32MultiArray()
    position.data = []
    for i in range(5):
        position.data.append(random.uniform(0,2*math.pi))
    pub=rospy.Publisher('/joint_states',Float32MultiArray,queue_size=10)
    rospy.init_node('talker',anonymous=True)
    rate=rospy.Rate(5)
    while not rospy.is_shutdown():
        pub.publish(position)
        rate.sleep()

if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
