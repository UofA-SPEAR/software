#!/usr/bin/env python
from actionlib import GoalStatus
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult
import rospy


class TimedMoveState(EventState):
    '''
    Send twist messages for a fixed period of time.

    -- linear float    Linear speed in m/s (positive is forward)
    -- angular float   Angular speed in rad/s (positive is left)
    -- duration float  Duration of the move

    <= ok              Indicates completed move with no errors

    '''
    def __init__(self, linear, angular, duration):
        super(TimedMoveState, self).__init__(outcomes=['ok'])
        self._publisher = rospy.Publisher('/drive_controller/cmd_vel', Twist, queue_size=1)
        self._start_time = None
        self._twist_message = Twist()
        self._twist_message.linear.x = linear
        self._twist_message.angular.z = angular
        self._duration = duration
        self.set_rate(10)

    def execute(self, ud):
        self._publisher.publish(self._twist_message)
        if rospy.get_time() - self._start_time >= self._duration:
            return 'ok'

    def on_enter(self, userdata):
        self._start_time = rospy.get_time()

    def on_exit(self, userdata):
        self._publisher.publish(Twist())
