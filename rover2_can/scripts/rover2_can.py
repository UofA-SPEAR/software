#!/usr/bin/env python
#
# An interface between ros messages and the uavcan protocol.

from __future__ import print_function
import canros
from drive_system.msg import drive_cmd
from functools import partial
import rospy


def ros_drive_cmd_callback(can_pub, can_msg, data):
    rospy.loginfo("[drive_cmd] Received: left = %d, right = %d" % (data.left,
                                                                   data.right))
    rospy.loginfo("[drive_cmd] Sending UAVCAN message...")

    # TODO: Map left & right from the ros message to the wheel number & speed
    #       in the UAVCAN message somehow...
    can_msg.wheel = 2
    can_msg.speed = 783
    can_pub.publish(can_msg)


def main():
    rospy.init_node("rover2_can")

    ####################################
    # Set up ROS -> UAVCAN subscribers #
    ####################################

    # drive_cmd
    can_drive = canros.Message("spear.drive.DriveCommand")
    rospy.Subscriber(
        "/drive", drive_cmd,
        partial(ros_drive_cmd_callback, can_drive.Publisher(queue_size=10),
                can_drive.Type()))

    rospy.spin()


if __name__ == '__main__':
    main()
