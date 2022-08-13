#! /usr/bin/env python3
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image, NavSatFix
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

# Instantiate CvBridge
bridge = CvBridge()


def gps_callback(fix):
    global latitude, longitude
    latitude = fix.latitude
    longitude = fix.longitude



def image_callback(msg):
    print("Received an image")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        time = msg.header.stamp
        cv2.imwrite(str(latitude), + "," + str(longitude) + ".jpg", cv2_img)
        rospy.sleep(1.)

def main():
    rospy.init_node('image_listener')
    # Define your image topic

    # Set up your subscriber and define its callback
    rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)
    rospy.Subscriber("gps/fix", NavSatFix, gps_callback)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()