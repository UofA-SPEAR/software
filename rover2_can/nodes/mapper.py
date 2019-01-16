#!/usr/bin/env python
#
# An interface between ros messages and the uavcan protocol.

from drive_system.msg import drive_cmd
import rospy
from rover2_can import map_ros_to_can, map_can_to_ros
from rover2_can.msg import DriveStatus, JointStatus, NodeStatus


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

    ####################################
    # Set up UAVCAN -> ROS subscribers #
    ####################################

    map_can_to_ros("spear.arm.JointStatus", JointStatus,
                   "/rover2_can/JointStatus", {
                       "joint": lambda data: data.joint,
                       "angle": lambda data: data.angle
                   })

    map_can_to_ros("spear.drive.DriveStatus", DriveStatus,
                   "/rover2_can/DriveStatus", {
                       "wheel": lambda data: data.wheel,
                       "speed": lambda data: data.speed
                   })

    map_can_to_ros(
        "uavcan.protocol.NodeStatus", NodeStatus, "/rover2_can/NodeStatus", {
            "uptime_sec": lambda data: data.uptime_sec,
            "mode": lambda data: data.mode,
            "health": lambda data: data.health,
            "node_id": lambda data: data.canros_uavcan_id
        })

    rospy.spin()


if __name__ == '__main__':
    main()
