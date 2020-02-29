import numpy as np
from flexbe_core import EventState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rospy import Publisher

from util.math import quaternion_to_yaw, angular_diff
from util.subscriber_value import SubscriberValue


class TurnCorrectionState(EventState):
    """
    Turn to face some target orientation.

    ># target_orientation QuaternionStamped    Orientation to turn to

    <= ok  Turn is complete
    """
    def __init__(self):
        super(TurnCorrectionState, self).__init__(outcomes=['ok'], input_keys=['target_orientation'])
        self._twist_topic = '/rover_diff_drive_controller/cmd_vel'
        self._odom_topic = '/ekf/odom/odometry/filtered'
        self._twist_publisher = Publisher(self._twist_topic, Twist, queue_size=1)
        self._odometry = SubscriberValue(self._odom_topic, Odometry)
        self._target_orientation = None  # type: QuaternionStamped

        self.STOPPING_ANGULAR_DISTANCE = 0.05
        self.MAX_ANGULAR_SPEED = 6.28
        self.PROPORTIONAL_CONSTANT = 6.0

    def on_enter(self, userdata):
        self._target_orientation = userdata.target_orientation

    def execute(self, ud):
        odometry = self._odometry.value  # type: Odometry
        assert odometry.header.frame_id == self._target_orientation.header.frame_id

        rover_angle = quaternion_to_yaw(odometry.pose.pose.orientation)
        target_angle = quaternion_to_yaw(self._target_orientation.quaternion)
        diff = angular_diff(target_angle, rover_angle)

        if abs(diff) < self.STOPPING_ANGULAR_DISTANCE:
            return 'ok'

        t = Twist()
        t.angular.z = diff * self.PROPORTIONAL_CONSTANT
        if abs(t.angular.z) > self.MAX_ANGULAR_SPEED:
            t.angular.z = self.MAX_ANGULAR_SPEED * np.sign(t.angular.z)
        self._twist_publisher.publish(t)

    def on_exit(self, userdata):
        self._twist_publisher.publish(Twist())
