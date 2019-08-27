#!/usr/bin/python

import rospy
from time import sleep, time
from geometry_msgs.msg import Twist


def main():
    rospy.init_node('move_with_twist')
    action = rospy.get_param('~action', 'forward')
    duration = rospy.get_param('~duration', '5')
    duration = float(duration)
    frequency = 10

    import sys
    rospy.loginfo(sys.argv)

    publisher = rospy.Publisher('/rover_diff_drive_controller/cmd_vel', Twist)

    message = Twist()
    if action == 'forward':
        message.linear.x = 1
    elif action == 'reverse':
        message.linear.x = -1
    elif action == 'left':
        message.angular.z = 0.5
    elif action == 'right':
        message.angular.z = -0.5

    rospy.loginfo('Movement started: %s.' % str(message))
    initial_time = time()
    while time() - initial_time < duration:
        publisher.publish(message)
        sleep(1)

    message = Twist()

    message.linear.x = 0

    rospy.loginfo('Movement started: %s.' % str(message))
    initial_time = time()
    while time() < initial_time + 1:
        publisher.publish(message)
        sleep(1)

    return True

    rospy.loginfo('Movement finished.')


if __name__ == '__main__':
    main()
