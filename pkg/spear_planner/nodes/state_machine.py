#!/usr/bin/env python3
from math import pi

import rospy
from geometry_msgs.msg import Twist
from rospy import Duration, Publisher, Time
from rospy.timer import Rate
from smach import Sequence, State

CMD_VEL_TOPIC = '/drive_controller/cmd_vel'


class TwistState(State):
    def __init__(self, linear_vel: float, angular_vel: float):
        super().__init__(outcomes=['ok', 'preempted'])
        self.cmd_vel_pub = Publisher(CMD_VEL_TOPIC, Twist, queue_size=1)
        self.rate = Rate(10)
        self.duration = Duration(1)
        self.linear_vel = linear_vel
        self.angular_vel = angular_vel

    def execute(self, ud):
        start_time = Time.now()
        while True:
            if rospy.is_shutdown():
                return 'preempted'
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            if Time.now() - start_time >= self.duration:
                return 'ok'

            self.cmd_vel_pub.publish(self.command())
            self.rate.sleep()

    def command(self):
        twist = Twist()
        twist.linear.x = self.linear_vel
        twist.angular.z = self.angular_vel
        return twist


def forward():
    return TwistState(linear_vel=1, angular_vel=0)


def turn():
    return TwistState(linear_vel=0, angular_vel=pi / 4)


def main():
    # Make a placeholder state machine for now. Can replace with something
    # interesting later.
    rospy.init_node('state_machine')
    sm = Sequence(outcomes=['preempted'], connector_outcome='ok')
    with sm:
        Sequence.add('forward', forward())
        Sequence.add('turn', turn(), {'ok': 'forward'})

    sm.execute()


if __name__ == '__main__':
    main()
