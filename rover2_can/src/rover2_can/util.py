# Utility functions.

import canros
from functools import partial
import rospy


def map_ros_to_can(ros_msg, ros_topic, can_msg, *mappings):
    """
    Subscribes to the specified ros_topic & message type, maps ros message data
    from ros's format to UAVCAN's format using mapping functions specified in
    "mapping".

    For example, imagine that you have a UAVCAN message named
    `spear.led.BlinkCommand`, that takes a `blink_frequency` parameter, and a
    ros message published to the `/led` topic called `LedPeriod` with a
    parameter `period`. Someone told you to map the ros message's `period`
    parameter to the UAVCAN message's `blink_frequency` parameter. Instead of
    a whole lot of spaghetti code, you could do that with this function like
    this:

        map_ros_to_can(LedPeriod, "/led", "spear.led.BlinkCommand", {
            "blink_frequency": lambda data: 1 / data.period
        })

    Parameters:

        - `ros_msg`: The rospy-generated datatype for the ros message you want
        to subscribe to.

        - `ros_topic`: A string. The name of the ros topic you want to
        subscribe to, in the same format that the rest of ros uses.

        - `can_msg`: A string. The name of the UAVCAN message you want to
        publish, in the same format as described in UAVCAN's docs.

        - `*mappings`: Dictionaries. Keys are the UAVCAN message parameters.
        Values are functions that take one value (the ros message data) and
        return a single value, which gets assigned to that UAVCAN message
        parameter.
        If more than one mapping is passed, like with:

            map_ros_to_can(RosMsg, "/rostopic", "uavcan.message", { ... mapping 1 ... }, { ... mapping 2 ... }, ...)

        ...then each mapping will receive the same RosMsg data, and each one
        will publish a unique message to "uavcan.message". This is to deal
        with cases where you might want to "split up" a single ros message into
        multiple UAVCAN messages (of the same type), without wanting to create
        several ros message subscribers (therefore saving memory).
    """

    def cb(can_pub, can_msg, mappings, data):
        # To reduce t

        # Loop over the mappings & call the functions within to translate data from
        # the ros message to data on the can message.
        #
        # The keys of the mapping dictionary are used as the property name on the can message.
        for mapping in mappings:
            for can_msg_param, ros_msg_mapper in mapping.iteritems():
                setattr(can_msg, can_msg_param, ros_msg_mapper(data))
            can_pub.publish(can_msg)

    rospy.loginfo(
        "Mapping ros message {} on topic {} to UAVCAN message {}.".format(
            ros_msg._type, ros_topic, can_msg))

    _can_msg = canros.Message(can_msg)

    rospy.Subscriber(
        ros_topic, ros_msg,
        partial(cb, _can_msg.Publisher(queue_size=10), _can_msg.Type(),
                mappings))


def map_can_to_ros(uavcan_message_type, ros_message, ros_topic, *mappings):
    rospy.loginfo(
        "Mapping UAVCAN message {} to ros message {} on topic {}.".format(
            uavcan_message_type, ros_message._type, ros_topic))

    can_msg = canros.Message(uavcan_message_type)
    pub = rospy.Publisher(ros_topic, ros_message, queue_size=10)

    def cb(data):
        for mapping in mappings:
            message_payload = {
                key: mapper(data)
                for (key, mapper) in mapping.iteritems()
            }
            pub.publish(**message_payload)

    can_msg.Subscriber(callback=cb)
