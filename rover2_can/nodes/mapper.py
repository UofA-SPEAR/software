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
        "wheel": lambda data: 0,
        "speed": lambda data: 783
    }, {
        "wheel": lambda data: 1,
        "speed": lambda data: 42
    }, {
        "wheel": lambda data: 2,
        "speed": lambda data: 123
    }, {
        "wheel": lambda data: 3,
        "speed": lambda data: 987
    })

    rospy.spin()


if __name__ == '__main__':
    main()
