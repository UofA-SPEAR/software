#!/usr/bin/env python
#
# An interface between ros messages and the uavcan protocol.

from canros import Message as CanrosMessage

from spear_msgs.msg import WheelCmdArray
from canros.msg import uavcan__equipment__actuator__ArrayCommand as ArrayCommand
from canros.msg import uavcan__equipment__actuator__Status as ActuatorStatus
from canros.msg import uavcan__equipment__power__BatteryInfo as BatteryInfo
from canros.msg import uavcan__protocol__NodeStatus as NodeStatus
from canros.msg import spear__general__PpmMessage as PpmMessage
import rospy
# How do you make it span multiple lines?
from spear_rover import map_ros_to_can, map_can_to_ros, test_bit

def arm_joint_mapper(data):
    out = []
    for command in data.commands:
        uavcan_msg = CanrosMessage('uavcan.equipment.actuator.Command')

        # The IDs of the arm's actuators start at 10 and run up to 15, with
        # the order defined by the rover2 arm_ik node. The id's in data.joints
        # start at 0 and run up to 5. So, we just add 10 to each data.joints
        # id.
        uavcan_msg.actuator_id = command.actuator_id + 10

        uavcan_msg.command_type = 1  # a.k.a. COMMAND_TYPE_POSITION
        uavcan_msg.command_value = command.command_value
        out.append(uavcan_msg)

    return out

def wheel_cmd_array_mapper(data):
    out = []
    for command in data.commands:
        uavcan_msg = CanrosMessage("uavcan.equipment.actuator.Command")
        uavcan_msg.actuator_id = command.actuator_id
        uavcan_msg.command_type = command.command_type  # a.k.a. COMMAND_TYPE_SPEED
        uavcan_msg.command_value = command.command_value
        out.append(uavcan_msg)



    return out


def arm_angles_mapper(data):
    out = []
    i = 0
    for command in data.commands:
        uavcan_msg = CanrosMessage('uavcan.equipment.actuator.Command')

        # The IDs of the arm's actuators start at 10 and run up to 15, with
        # the order defined by the rover2 arm_ik node. The id's in data.joints
        # start at 0 and run up to 5. So, we just add 10 to each data.joints
        # id.
        uavcan_msg.actuator_id = command.actuator_id + 10

        uavcan_msg.command_type = 1  # a.k.a. COMMAND_TYPE_POSITION
        uavcan_msg.command_value = data.position[i]
        out.append(uavcan_msg)
        i += 1

    return out


def main():
    rospy.init_node("can_mapper")

    ####################################
    # Set up ROS -> UAVCAN subscribers #
    ####################################

    map_ros_to_can(ArrayCommand, '/drive/cmds',
                   'uavcan.equipment.actuator.ArrayCommand',
                   {"commands": wheel_cmd_array_mapper})

    #map_ros_to_can(ArrayCommand, '/arm/angles',
    #               'uavcan.equipment.actuator.ArrayCommand',
    #               {'commands': arm_angles_mapper})


    # Get radians per joint from topic
    # Append to ArmArrayCommand



    # Iterate through /joint_state
    map_ros_to_can(ArrayCommand, '/joint_state',
		           'uavcan.equipment.actuator.ArrayCommand',
		           {'commands': arm_angles_mapper})

    ####################################
    # Set up UAVCAN -> ROS subscribers #
    ####################################

    map_can_to_ros("spear.general.PpmMessage", PpmMessage,
                   "/rover/can/PpmMessage",
                   {"channel_data": lambda data: data.channel_data})

    map_can_to_ros(
        "uavcan.equipment.actuator.Status", ActuatorStatus,
        "/rover/can/ActuatorStatus", {
            "actuator_id": lambda data: data.actuator_id,
            "position": lambda data: data.position,
            "force": lambda data: data.force,
            "speed": lambda data: data.speed,
            "power_rating_pct": lambda data: data.power_rating_pct
        })

    map_can_to_ros(
        "uavcan.equipment.power.BatteryInfo",
        BatteryInfo,
        "/rover/can/BatteryInfo",
        {
            "temperature": lambda data: data.temperature,
            "voltage": lambda data: data.voltage,
            "current": lambda data: data.current,
            "average_power_10sec": lambda data: data.average_power_10sec,
            "remaining_capacity_wh": lambda data: data.remaining_capacity_wh,
            "hours_to_full_charge": lambda data: data.hours_to_full_charge,

            # The UAVCAN BatteryInfo message uses a bitmask for its
            # status_flags field. While we could potentially use one as well,
            # it would be a lot easier in the long run if we could just access
            # them as individual bools. So here we check for each flag.
            "in_use": lambda data: test_bit(data.status_flags, 0),
            "charging": lambda data: test_bit(data.status_flags, 1),
            "charged": lambda data: test_bit(data.status_flags, 2),
            "temp_hot": lambda data: test_bit(data.status_flags, 3),
            "temp_cold": lambda data: test_bit(data.status_flags, 4),
            "overload": lambda data: test_bit(data.status_flags, 5),
            "bad_battery": lambda data: test_bit(data.status_flags, 6),
            "need_service": lambda data: test_bit(data.status_flags, 7),
            "bms_error": lambda data: test_bit(data.status_flags, 8),

            # State of health and charge.
            "state_of_health_pct": lambda data: data.state_of_health_pct,
            "state_of_charge_pct": lambda data: data.state_of_charge_pct,
            "state_of_charge_pct_stdev": lambda data: data.
            state_of_charge_pct_stdev,

            # Battery identification.
            "battery_id": lambda data: data.battery_id,
            "model_instance_id": lambda data: data.model_instance_id,
            "model_name": lambda data: data.model_name
        })

    map_can_to_ros(
        "uavcan.protocol.NodeStatus", NodeStatus, "/rover/can/NodeStatus", {
            "uptime_sec": lambda data: data.uptime_sec,
            "mode": lambda data: data.mode,
            "health": lambda data: data.health,
        })

    rospy.spin()


if __name__ == '__main__':
    main()
