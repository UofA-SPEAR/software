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
    while True:
        publisher.publish(message)
        if (time() - initial_time) > duration:
            break
        sleep(1/frequency)
    
    initial_time = time()
    while (time() - initial_time) < 1:
        publisher.publish(Twist())

    rospy.loginfo('Movement finished.')


if __name__ == '__main__':
    main()
