#!/usr/bin/env python

import os
import pytesseract
import rospy
from PIL import Image as PIL_Image
from cv_bridge import CvBridge, CvBridgeError
#import image_transport
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2

pub = None


# Extract text fromm the image sent through ROS
def image_to_text(image, preprocess):

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # check to see if we should apply thresholding to preprocess the
    # image
    if preprocess == "thresh":
        gray = cv2.threshold(
            gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU
        )[1]  # performing threshold to segment foreground from background

    # make a check to see if median blurring should be done to remove
    # noise
    elif preprocess == "blur":
        gray = cv2.medianBlur(gray, 3)

    # write the grayscale image to disk as a temporary file so we can
    # apply OCR to it
    filename = "/tmp/{}.png".format(os.getpid())
    cv2.imwrite(filename, gray)

    text = pytesseract.image_to_string(PIL_Image.open(filename))

    return text


def callback(msg):
    # convert ros image to opencv image
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    text = image_to_text(image, "blur")
    pub.publish(str(text))


def main():
    global pub
    pub = rospy.Publisher("text", String, queue_size=10)
    rospy.init_node("ocr_node", anonymous=True)
    rospy.Subscriber("/camera_topic", Image, callback)
    rospy.spin()


if __name__ == "__main__":
    main()
