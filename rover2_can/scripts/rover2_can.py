#!/usr/bin/env python
#
# An interface between ros messages and the uavcan protocol.

from __future__ import print_function
import canros
import rospy

rospy.init_node("rover2_can")

## TEST ##
drive = canros.Message("spear.drive.DriveCommand")
drive_pub = drive.Publisher(queue_size=10)
drive_msg = drive.Type()
drive_msg.wheel = 2
drive_msg.speed = 783

# Spin while publishing the message every second.
while True:
    rospy.sleep(1)
    if rospy.is_shutdown():
        raise Exception("ROS shutdown")
    drive_pub.publish(drive_msg)
