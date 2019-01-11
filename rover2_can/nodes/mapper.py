#!/usr/bin/env python
#
# An interface between ros messages and the uavcan protocol.

from drive_system.msg import drive_cmd
import rospy
from rover2_can import *


def main():
    rospy.init_node("rover2_can")

    ####################################
    # Set up ROS -> UAVCAN subscribers #
    ####################################

    # TODO: Map left & right from the ros message to the wheel number & speed
    #       in the UAVCAN message somehow...
    map_ros_to_can(drive_cmd, "/drive", "spear.drive.DriveCommand", {
        "wheel": lambda data: 2,
        "speed": lambda data: 783
    })

    rospy.spin()


if __name__ == '__main__':
    main()
