#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge, CvBridgeError
#import image_transport
from sensor_msgs.msg import Image

import cv2

def callback(msg):
    # convert ros image to opencv image
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


def main():
    rospy.init_node("ocr_node", anonymous=True)
    rospy.Subscriber("/camera_topic", Image, callback)
    rospy.spin()


if __name__ == "__main__":
    main()
