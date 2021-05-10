#!/usr/bin/env python3
#
# A node which reads diagnostics information from various sources and sends them
# to spearMCT

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from abc import ABCMeta


class DiagnosticPublisher:
    def __init__(self, topic, data_class):
        self.data_class = data_class
        self.publisher = rospy.Publisher(topic, data_class, queue_size=1)

    def publish(self, data):
        message = self.data_class()
        message.data = data
        self.publisher.publish(message)


twist_forward_publisher = DiagnosticPublisher('web/twist/forward', Float64)
twist_turn_publisher = DiagnosticPublisher('web/twist/turn', Float64)

twist_publisher = rospy.Publisher('web/twist', Twist, queue_size=1)


def on_twist(message):  # type: (Twist) -> None
    twist_forward_publisher.publish(message.linear.x)
    twist_turn_publisher.publish(message.angular.z)


def main():
    rospy.init_node('diagnostics')
    s = rospy.Subscriber('/drive_controller/cmd_vel', Twist, on_twist)
    rospy.spin()


if __name__ == '__main__':
    main()
