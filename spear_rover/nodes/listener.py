#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def listener():
	# creates & registers new ROS node
	# subscribes it to my_topic (same topic name published to)
	rospy.init_node("listener", anonymous=True)
	rospy.Subscriber("my_topic", String, callback)

	# keeps code running, listening for msgs
	rospy.spin()

if __name__ == "__main__":
	listener()