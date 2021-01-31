#!/usr/bin/env python
import math
from tempfile import TemporaryFile
from typing import Dict, List, Optional

import ikpy
import numpy as np
import pygame
import rospy
import tf2_geometry_msgs
import tf2_ros
from control_msgs.srv import QueryTrajectoryState
from geometry_msgs.msg import PointStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import transforms3d
from geometry_msgs.msg import TransformStamped, Transform, Vector3, _Vector3Stamped
from spear_station import transform_to_matrix, lookup_transform_simple


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


class IK:
    def __init__(self, robot_description, valid_joints,
                 base_joint):  # type: (str, List[str], str) -> None
        self.valid_joints = valid_joints
        self.base_joint = base_joint
        with TemporaryFile() as file:
            file.write(robot_description)
            file.seek(0)
            self.chain = ikpy.chain.Chain.from_urdf_file(file, [base_joint])

    def backward(self, target_position):  # type: (np.ndarray) -> JointAngles
        angles = self.chain.inverse_kinematics(target_position)
        joint_names = [link.name for link in self.chain.links]
        return JointAngles({
            name: angle
            for name, angle in zip(joint_names, angles)
            if name in self.valid_joints
        })

    def forward(self, joint_angles):
        joint_names = [link.name for link in self.chain.links]
        joint_angles = [joint_angles.get(name, 0) for name in joint_names]
        target_matrix = self.chain.forward_kinematics(joint_angles)
        return target_matrix

    def frame(self):
        return self.base_joint


class ArmTarget:
    def __init__(self, frame_id):  # type: (str) -> None
        self._frame_id = frame_id
        self._yaw = 0
        self._pitch = 0
        self._radius = 1
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

    def cw(self, angle):  # type: (float) -> None
        self._yaw -= angle

    def ccw(self, angle):  # type: (float) -> None
        self._yaw += angle

    def up(self, angle):  # type: (float) -> None
        self._pitch += angle

    def down(self, angle):  # type: (float) -> None
        self._pitch -= angle

    def retract(self, value):  # type: (float) -> None
        self._radius -= value

    def expand(self, value):  # type: (float) -> None
        self._radius += value

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
            transform = lookup_transform_simple(self._tf_buffer,
                                                target_frame=frame_id,
                                                source_frame=self._frame_id)
            point = tf2_geometry_msgs.do_transform_point(point, transform)
        return point


class HandTarget:
    def __init__(self):
        self._pitch = 0
        self._roll = 0
        self._grip = 0

    def up(self, angle):
        self._pitch -= angle

    def down(self, angle):
        self._pitch += angle

    def cw(self, angle):
        self._roll += angle

    def ccw(self, angle):
        self._roll -= angle

    def tighten(self, angle):
        self._grip += angle

    def loosen(self, angle):
        self._grip -= angle

    def goal(self):
        return self._pitch, self._roll, self._grip


class HandPlanner:
    def __init__(self, pitch_joint, roll_joint,
                 grip_joint):  # type: (str, str, str) -> None
        self.pitch_joint = pitch_joint
        self.roll_joint = roll_joint
        self.grip_joint = grip_joint

    def angles(self, pitch_roll_grip):
        pitch, roll, grip = pitch_roll_grip
        return JointAngles({
            self.pitch_joint: pitch,
            self.roll_joint: roll,
            self.grip_joint: grip,
        })


rospy.init_node('arm_ik')
rospy.sleep(0)

command_publisher = rospy.Publisher('/arm_controller/command',
                                    JointTrajectory,
                                    queue_size=1)
target_point_publisher = rospy.Publisher('/viz/arm_target',
                                         PointStamped,
                                         queue_size=1)

# rospy.wait_for_service('/arm_controller/query_state')
# query_trajectory_state = rospy.ServiceProxy('/arm_controller/query_state',
#                                             QueryTrajectoryState)
# joint_names = query_trajectory_state(rospy.Time.now()).name
hand_joint_names = ['wrist_pitch', 'wrist_roll', 'grab']
arm_joint_names = ['shoulder_yaw', 'shoulder_pitch', 'elbow_pitch']

robot_description = rospy.get_param('robot_description')  # type: str
ik = IK(robot_description, arm_joint_names, 'arm_mount')
arm_target = ArmTarget('arm_mount')

hand_target = HandTarget()
hand_planner = HandPlanner(*hand_joint_names)

arm_target.up(-math.pi / 6)
arm_target.cw(math.pi)
arm_target.retract(0.5)

pygame.init()
pygame.display.set_mode((100, 100))
pygame.display.set_caption('IK Controller')


def key_pressed(key):  # type: (str) -> bool
    index = pygame.__dict__['K_' + key]
    return pygame.key.get_pressed()[index]


tf_buffer = arm_target._tf_buffer

angle_locked = True
angle_target = 3.14 / 2

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    arm_angles = ik.backward(arm_target.position(ik.frame()))
    hand_angles = hand_planner.angles(hand_target.goal())
    trajectory = arm_angles.combine_with(hand_angles).to_command(
        rospy.Duration(0.01))
    command_publisher.publish(trajectory)
    target_point_publisher.publish(arm_target.point())

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            break
        if event.type == pygame.KEYDOWN and event.key == pygame.K_l:
            angle_locked = not angle_locked
            if angle_locked:
                hand_to_base_link = transform_to_matrix(
                lookup_transform_simple(tf_buffer, 'base_link', 'forearm'))
                _, hand_to_base_link_rot, _, _ = transforms3d.affines.decompose44(
                    hand_to_base_link)

                x = np.asarray([0, 0, 1])

                x_transformed = np.matmul(hand_to_base_link_rot, x)
                angle_target = np.arccos(np.clip(np.dot([0, 0, 1], x_transformed), -1.0, 1.0))
    if key_pressed('a'):
        arm_target.ccw(0.05)
    if key_pressed('d'):
        arm_target.cw(0.05)
    if key_pressed('w'):
        arm_target.up(0.05)
    if key_pressed('s'):
        arm_target.down(0.05)
    if key_pressed('e'):
        arm_target.expand(0.01)
    if key_pressed('q'):
        arm_target.retract(0.01)
    if key_pressed('j'):
        hand_target.ccw(0.05)
    if key_pressed('l'):
        hand_target.cw(0.05)
    if key_pressed('i'):
        hand_target.up(0.05)
    if key_pressed('k'):
        hand_target.down(0.05)
    if key_pressed('u'):
        hand_target.tighten(0.01)
    if key_pressed('o'):
        hand_target.loosen(0.01)

    if angle_locked:  # Maintain hand pitch relative to base
        # arm_mount_to_hand = ik.forward(arm_angles)
        # base_link_to_arm_mount = transform_to_matrix(
        #     lookup_transform_simple(tf_buffer, 'base_link', ik.frame()))
        # base_link_to_hand = np.matmul(base_link_to_arm_mount,
        #                               arm_mount_to_hand)
        # hand_to_base_link = np.linalg.inv(base_link_to_hand)
        hand_to_base_link = transform_to_matrix(
            lookup_transform_simple(tf_buffer, 'base_link', 'forearm'))
        _, hand_to_base_link_rot, _, _ = transforms3d.affines.decompose44(
            hand_to_base_link)

        x = np.asarray([0, 0, 1])

        x_transformed = np.matmul(hand_to_base_link_rot, x)
        angle = np.arccos(np.clip(np.dot([0, 0, 1], x_transformed), -1.0, 1.0))
        hand_target._pitch = angle_target - angle

    rate.sleep()
