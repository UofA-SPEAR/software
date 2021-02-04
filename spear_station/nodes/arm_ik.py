#!/usr/bin/env python
import math
from tempfile import TemporaryFile
from typing import Callable, Dict, Generic, Iterable, Optional, TypeVar

import ikpy
import numpy as np
import pygame
import rospy
import transforms3d
from geometry_msgs.msg import PointStamped
from spear_station import (do_transform_msg, get_urdf, lookup_transform_simple,
                           transform_to_matrix)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JointAngles:
    def __init__(self, angles):  # type: (Dict[str, float]) -> None
        self._angles = angles

    def to_command(
            self,
            time_from_start):  # type: (rospy.Duration) -> JointTrajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = self._angles.keys()
        trajectory.points = [JointTrajectoryPoint()]
        trajectory.points[0].positions = self._angles.values()
        trajectory.points[0].time_from_start = time_from_start
        return trajectory

    def combine_with(self, other):  # type: (JointAngles) -> JointAngles
        common_keys = set(self._angles.keys()).intersection(
            other._angles.keys())
        if common_keys:
            rospy.logwarn('Redundant joints in JointAngles.combine: {}'.format(
                common_keys))
        angles = self._angles.copy()
        angles.update(other._angles)
        return JointAngles(angles)

    def get(self, joint_name, default=None):
        return self._angles.get(joint_name, default)

    @staticmethod
    def combine(angles):  # type: (Iterable[JointAngles]) -> JointAngles
        angles = list(angles)
        if len(angles) == 0:
            raise ValueError("Cannot combine []")
        if len(angles) == 1:
            return angles[0]
        return angles[0].combine_with(JointAngles.combine(angles[1:]))


class IKPlanner:
    def __init__(self, base_link, tip_link):  # type: (str, str) -> None
        self._base_link = base_link
        self._tip_link = tip_link
        self._chain = self._get_ikpy_chain(self._base_link)
        self._valid_joints = get_urdf().get_chain(base_link,
                                                  tip_link,
                                                  links=False)

    @staticmethod
    def _get_ikpy_chain(base_link):  # type: (str) -> ikpy.chain.Chain
        robot_description = rospy.get_param('robot_description')
        with TemporaryFile() as file:
            file.write(robot_description)
            file.seek(0)
            return ikpy.chain.Chain.from_urdf_file(file, [base_link])

    def angles(self, target_position):  # type: (np.ndarray) -> JointAngles
        angles = self._chain.inverse_kinematics(target_position)
        joint_names = [link.name for link in self._chain.links]
        return JointAngles({
            name: angle
            for name, angle in zip(joint_names, angles)
            if name in self._valid_joints
        })

    def frame(self):
        return self._base_link


class PointTarget:
    def __init__(self, frame_id):  # type: (str) -> None
        self._frame_id = frame_id
        self._yaw = 0
        self._pitch = 0
        self._radius = 1

    def position(self, frame_id=None):  # type: (Optional[str]) -> None
        point = self.point(frame_id)
        return np.array([point.point.x, point.point.y, point.point.z])

    def point(self, frame_id=None):  # type: (Optional[str]) -> PointStamped
        point = PointStamped()
        point.header.frame_id = self._frame_id
        point.point.x, point.point.y, point.point.z = np.array([
            np.cos(self._yaw) * np.cos(self._pitch),
            np.sin(self._yaw),
            np.sin(self._pitch)
        ]) * self._radius

        if frame_id is not None:
            point = do_transform_msg(point, target_frame=frame_id)
        return point


T = TypeVar("T")

class TwoWayBinding(Generic[T]):
    def __init__(
            self, get_fn,
            set_fn):  # type: (Callable[[], T], Callable[[T], None]) -> None
        self._get_fn = get_fn
        self._set_fn = set_fn

    @property
    def value(self):
        return self._get_fn()

    @value.setter
    def value(self, x):
        self._set_fn(x)

    @classmethod
    def from_member(cls, object_, name):  # type: (object, str) -> TwoWayBinding
        return cls(get_fn=lambda: getattr(object_, name),
                   set_fn=lambda value: setattr(object_, name, value))


class IKController:
    def __init__(self, base_link, tip_link,
                 target_link):  # type: (str, str, str) -> None
        self._planner = IKPlanner(base_link, tip_link)
        self._target = PointTarget(target_link)

    def angles(self):
        return self._planner.angles(
            self._target.position(self._planner.frame()))

    def yaw_binding(self):
        return TwoWayBinding.from_member(self._target, '_yaw')

    def pitch_binding(self):
        return TwoWayBinding.from_member(self._target, '_pitch')

    def radius_binding(self):
        return TwoWayBinding.from_member(self._target, '_radius')


class ManualJointController:
    def __init__(self, joint):  # type: (str) -> None
        self._joint = joint
        self._angle = 0

    def angles(self):
        return JointAngles({self._joint: self._angle})

    def binding(self):
        return TwoWayBinding.from_member(self, '_angle')


class LockedJointController:
    def __init__(self, base_frame, joint,
                 axis):  # type: (str, str, np.ndarray) -> None
        """
        This probably requires some explanation. The idea is to control a joint,
        but allow the angle (relative to some axes) of the parent link to be
        "locked" in place. Let T be the transformation from the parent link to
        `base_frame`. Then the angle being "locked" is the angle
        between T(axis) and axis.
        """
        self._base_frame = base_frame
        self._target_frame = get_urdf().joint_map[joint].parent
        self._joint_name = joint
        self._axis = axis
        self._target_angle = 0

    def _get_actual_angle(self):
        T = transform_to_matrix(
            lookup_transform_simple(target_frame=self._base_frame,
                                    source_frame=self._target_frame))
        _, T_rot, _, _ = transforms3d.affines.decompose44(T)
        transformed_axis = np.matmul(T_rot, self._axis)
        return self._angle_between(self._axis, transformed_axis)

    def _angle_between(self, u, v):
        u /= np.linalg.norm(u)
        v /= np.linalg.norm(v)
        return np.arccos(np.dot(u, v))

    def angles(self):
        return JointAngles(
            {self._joint_name: self._target_angle - self._get_actual_angle()})

    def binding(self):
        return TwoWayBinding.from_member(self, '_target_angle')


class KeypressedView:
    def __init__(self, keys, model,
                 increment):  # type: (str, TwoWayBinding, float) -> None
        self._inc_key, self._dec_key = keys
        self._model = model
        self._increment = increment

    @staticmethod
    def _key_pressed(key):  # type: (str) -> bool
        index = pygame.__dict__['K_' + key]
        return pygame.key.get_pressed()[index]

    def tick(self):
        if self._key_pressed(self._inc_key):
            self._model.value += self._increment
        if self._key_pressed(self._dec_key):
            self._model.value -= self._increment


rospy.init_node('arm_ik')
rospy.sleep(0)

pygame.init()
pygame.display.set_mode((100, 100))
pygame.display.set_caption('IK Controller')

command_publisher = rospy.Publisher('/arm_controller/command',
                                    JointTrajectory,
                                    queue_size=1)
target_point_publisher = rospy.Publisher('/viz/arm_target',
                                         PointStamped,
                                         queue_size=1)

ik_controller = IKController('arm_mount', 'forearm', 'arm_mount')
wrist_controller = LockedJointController('base_link', 'wrist_pitch', [0, 0, 1])
hand_roll_controller = ManualJointController('wrist_roll')
hand_grab_controller = ManualJointController('grab')

controllers = [
    ik_controller,
    wrist_controller,
    hand_roll_controller,
    hand_grab_controller,
]

views = [
    KeypressedView('ws', ik_controller.pitch_binding(), 0.05),
    KeypressedView('ad', ik_controller.yaw_binding(), 0.05),
    KeypressedView('eq', ik_controller.radius_binding(), 0.01),
    KeypressedView('ki', wrist_controller.binding(), 0.05),
    KeypressedView('lj', hand_roll_controller.binding(), 0.05),
    KeypressedView('uo', hand_grab_controller.binding(), 0.1),
]

ik_controller.pitch_binding().value = -math.pi / 6
ik_controller.yaw_binding().value = math.pi
ik_controller.radius_binding().value = 0.5
wrist_controller.binding().value = math.pi / 2

rate = rospy.Rate(10)
running = True
while running and not rospy.is_shutdown():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    angles = JointAngles.combine(c.angles() for c in controllers)
    trajectory = angles.to_command(rospy.Duration(0.01))
    command_publisher.publish(trajectory)

    # XXX: a total hack, fix this
    target_point_publisher.publish(ik_controller._target.point())

    for view in views:
        view.tick()

    rate.sleep()
