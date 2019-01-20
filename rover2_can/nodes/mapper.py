#!/usr/bin/env python
#
# An interface between ros messages and the uavcan protocol.

from drive_system.msg import drive_cmd
import rospy
from rover2_can import map_ros_to_can, map_can_to_ros, test_bit
from rover2_can.msg import ActuatorStatus, BatteryInfo, NodeStatus


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

    map_can_to_ros(
        "uavcan.equipment.actuator.Status", ActuatorStatus,
        "/rover2_can/ActuatorStatus", {
            "actuator_id": lambda data: data.actuator_id,
            "position": lambda data: data.position,
            "force": lambda data: data.force,
            "speed": lambda data: data.speed,
            "power_rating_pct": lambda data: data.power_rating_pct
        })

    map_can_to_ros(
        "uavcan.equipment.power.BatteryInfo",
        BatteryInfo,
        "/rover2_can/BatteryInfo",
        {
            "temperature":
            lambda data: data.temperature,
            "voltage":
            lambda data: data.voltage,
            "current":
            lambda data: data.current,
            "average_power_10sec":
            lambda data: data.average_power_10sec,
            "remaining_capacity_wh":
            lambda data: data.remaining_capacity_wh,
            "hours_to_full_charge":
            lambda data: data.hours_to_full_charge,

            # The UAVCAN BatteryInfo message uses a bitmask for its status_flags
            # field. While we could potentially use one as well, it would be a lot
            # easier in the long run if we could just access them as individual
            # bools. So here we check for each flag.
            "in_use":
            lambda data: test_bit(data.status_flags, 0),
            "charging":
            lambda data: test_bit(data.status_flags, 1),
            "charged":
            lambda data: test_bit(data.status_flags, 2),
            "temp_hot":
            lambda data: test_bit(data.status_flags, 3),
            "temp_cold":
            lambda data: test_bit(data.status_flags, 4),
            "overload":
            lambda data: test_bit(data.status_flags, 5),
            "bad_battery":
            lambda data: test_bit(data.status_flags, 6),
            "need_service":
            lambda data: test_bit(data.status_flags, 7),
            "bms_error":
            lambda data: test_bit(data.status_flags, 8),

            # State of health and charge.
            "state_of_health_pct":
            lambda data: data.state_of_health_pct,
            "state_of_charge_pct":
            lambda data: data.state_of_charge_pct,
            "state_of_charge_pct_stdev":
            lambda data: data.state_of_charge_pct_stdev,

            # Battery identification.
            "battery_id":
            lambda data: data.battery_id,
            "model_instance_id":
            lambda data: data.model_instance_id,
            "model_name":
            lambda data: data.model_name
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
