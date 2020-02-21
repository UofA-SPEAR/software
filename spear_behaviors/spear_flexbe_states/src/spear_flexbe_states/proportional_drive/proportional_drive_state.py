import numpy as np
from flexbe_core import EventState
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
from rospy import Publisher

from util.subscriber_value import SubscriberValue
from ros_numpy import numpify
from util.math import quaternion_to_yaw, angular_diff


class ProportionalDriveState(EventState):
    """
    Drive directly to a target using a simple proportional controller.

    ># target_position PointStamped    Point to drive to

    <= ok  Drive is complete
    """
    def __init__(self):
        super(ProportionalDriveState, self).__init__(outcomes=['ok'], input_keys=['target_position'])
        self._twist_topic = '/rover_diff_drive_controller/cmd_vel'
        self._odom_topic = '/ekf/odom/odometry/filtered'
        self._visual_topic = '/viz/proportional_drive/target_position'
        self._twist_publisher = Publisher(self._twist_topic, Twist, queue_size=1)
        self._odometry = SubscriberValue(self._odom_topic, Odometry)
        self._visual_publisher = Publisher(self._visual_topic, PointStamped, queue_size=1)
        self._target_position = None  # type: PointStamped

        self.STOPPING_DISTANCE = 0.1
        self.FORWARD_SPEED = 1.0
        self.MAX_ANGULAR_SPEED = 6.28
        self.PROPORTIONAL_CONSTANT = 6.0

    def on_enter(self, userdata):
        self._target_position = userdata.target_position

    def execute(self, ud):
        self._visual_publisher.publish(self._target_position)

        odometry = self._odometry.value  # type: Odometry
        assert odometry.header.frame_id == self._target_position.header.frame_id

        rover_position2d = numpify(odometry.pose.pose.position)[:2]
        target_position2d = numpify(self._target_position.point)[:2]

        if np.linalg.norm(rover_position2d - target_position2d) < self.STOPPING_DISTANCE:
            return 'ok'

        rover_angle = quaternion_to_yaw(odometry.pose.pose.orientation)
        angle_to_target = np.arctan2(target_position2d[1]-rover_position2d[1], target_position2d[0]-rover_position2d[0])
        diff = angular_diff(angle_to_target, rover_angle)
        print(diff)

        t = Twist()
        if abs(diff) < np.pi/4:
            t.linear.x = self.FORWARD_SPEED
        t.angular.z = diff * self.PROPORTIONAL_CONSTANT
        if abs(t.angular.z) > self.MAX_ANGULAR_SPEED:
            t.angular.z = self.MAX_ANGULAR_SPEED * np.sign(t.angular.z)
        self._twist_publisher.publish(t)

    def on_exit(self, userdata):
        self._twist_publisher.publish(Twist())


