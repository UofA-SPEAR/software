from typing import Optional

import numpy as np
from geometry_msgs.msg import PoseStamped, PointStamped, Point, Vector3Stamped, Vector3
from ros_numpy import msgify, numpify

from ar_tracker import ARTracker
from util.math import quaternion_rotate


class Post:
    def __init__(self, marker_id, tracker):  # type: (int, ARTracker) -> None
        self._marker_id = marker_id
        self._tracker = tracker

    @property
    def pose(self):  # type: () -> Optional[PoseStamped]
        return self._tracker.get_pose(self._marker_id)

    @property
    def position(self):  # type: () -> Optional[PointStamped]
        if self.pose is None:
            return None
        point = PointStamped()
        point.header.frame_id = self.pose.header.frame_id
        point.point = self.pose.pose.position
        return point

    @property
    def normal(self):  # type: () -> Optional[Vector3Stamped]
        if self.pose is None:
            return None
        normal_vec = Vector3Stamped()
        normal_vec.header.frame_id = self.pose.header.frame_id
        normal_vec.vector = msgify(Vector3, quaternion_rotate(numpify(self.pose.pose.orientation), np.array([0.0, 0.0, 1.0])))
        return normal_vec

    def has_seen(self):  # type: () -> bool
        return self._tracker.has_seen(self._marker_id)


class Gate:
    def __init__(self, left_post, right_post):  # type: (Post, Post) -> None
        self._left_post = left_post
        self._right_post = right_post

    @property
    def position(self):  # type: () -> Optional[PointStamped]
        if self._left_post.pose is None or self._right_post.pose is None:
            return None
        assert self._left_post.position.header.frame_id == self._right_post.position.header.frame_id
        position = PointStamped()
        position.header.frame_id = self._left_post.pose.header.frame_id
        left_position = numpify(self._left_post.position.point)
        right_position = numpify(self._right_post.position.point)
        position.point = msgify(Point, (left_position + right_position) / 2)
        return position

    @property
    def normal(self):  # type: () -> Optional[Vector3Stamped]
        """
        If the rover is facing the gate head on, with the left post at the left
        and the right post at the right, the gate's surface normal points from
        the gate to the rover.
        """
        if self._left_post.pose is None or self._right_post.pose is None:
            return None
        assert self._left_post.position.header.frame_id == self._right_post.position.header.frame_id

        left_position = numpify(self._left_post.position.point)
        right_position = numpify(self._right_post.position.point)

        normal = np.cross(right_position - left_position, np.array([0.0, 0.0, 1.0]))
        normal /= np.linalg.norm(normal)

        normal_vec = Vector3Stamped()
        normal_vec.header.frame_id = self._left_post.position.header.frame_id
        normal_vec.vector = msgify(Vector3, normal)
        return normal_vec

    def has_seen(self):
        return self._left_post.has_seen() and self._right_post.has_seen()
