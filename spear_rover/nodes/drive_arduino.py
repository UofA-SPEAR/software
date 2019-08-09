#!/usr/bin/env python
#
# A node to send drive mesages over serial to an arduino

import serial

import rospy

from canros.msg import uavcan__equipment__actuator__ArrayCommand as ArrayCommand

# Declare serial object as global variable so that callback can access it
ser = None


def map_and_constrain(x, from_low, from_high, to_low, to_high):
    x = (x - from_low) * (to_high - to_low) / (from_high - from_low) + to_low
    if x > to_high:
        x = to_high
    elif x < to_low:
        x = to_low
    return x


def create_serial_msg(side, power):
    """Returns a bytearray to be written to serial"""
    side = ord(side)
    # Take modulo to avoid 8 bit overflow
    checksum = (side + power) % 256
    buf = bytearray([
        2,  # start transmission
        side,
        power,
        checksum,
        3
    ])  # end transmission
    return buf


def send_command(side, power):
    global ser
    """Writes a drive command to the provided serial"""
    buf = create_serial_msg(side, power)
    ser.write(buf)


def callback(msg):
    for cmd in msg.commands:
        side = 'L'

        if cmd.actuator_id == 2 or cmd.actuator_id == 3:
            side = 'L'
        elif cmd.actuator_id == 0 or cmd.actuator_id == 1:
            side = 'R'

        power = int(map_and_constrain(cmd.command_value, -1, 1, -50, 50))
        # 8 bit 2's complement
        if power < 0:
            power = int(256 + power)

        send_command(side, power)


def main():
    global ser
    rospy.init_node("drive_arduino")

    port = rospy.get_param("~serial_port")

    ser = serial.Serial(port, 9600, timeout=1)
    rospy.loginfo("Opened port: {}".format(ser.name))
    rospy.Subscriber("/drive/cmds", ArrayCommand, callback)
    rospy.spin()
    ser.close()


if __name__ == "__main__":
    main()
