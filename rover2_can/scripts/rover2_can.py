#!/usr/bin/env python
#
# An interface between ros messages and the uavcan protocol.

from __future__ import print_function
import canros
import rospy

rospy.init_node("rover2_can")

# Retrieve canros object for the message 'uavcan.protocol.enumeration.Indication'.
indication = canros.Message("uavcan.protocol.enumeration.Indication")

# Create a ROS Publisher using the canros object.
pub = indication.Publisher(queue_size=10)

# Use canros object to obtain the ROS type.
msg = indication.Type()

# Obtain the ROS type for the message 'uavcan.protocol.param.NumericValue',
# which is part of 'uavcan.protocol.enumeration.Indication'.
msg.value = canros.Message("uavcan.protocol.param.NumericValue").Type()

# 'uavcan.protocol.param.NumericValue' is a union type to the 'canros_union_tag' field needs to be set.
msg.value.canros_union_tag = msg.value.CANROS_UNION_TAG_REAL_VALUE
msg.value.real_value = 3.14159265

msg.parameter_name = canros.to_uint8("canros test")

# Spin while publishing the message every second.
while True:
    rospy.sleep(1)
    if rospy.is_shutdown():
        raise Exception("ROS shutdown")
    pub.publish(msg)
