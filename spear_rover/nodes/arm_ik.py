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
        joints = [link.name for link in self.chain.links]
        return JointAngles({
            name: angle
            for name, angle in zip(joints, angles) if name in self.valid_joints
        })

    def frame(self):
        return self.base_joint


class Target:
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
            while not rospy.is_shutdown():
                try:
                    transform = self._tf_buffer.lookup_transform(
                        target_frame=frame_id,
                        source_frame=self._frame_id,
                        time=rospy.Time(0),
                        timeout=rospy.Duration(0.1))
                    point = tf2_geometry_msgs.do_transform_point(
                        point, transform)
                    break
                except Exception as err:
                    rospy.logerr(err)
        return point


rospy.init_node('arm_ik')
rospy.sleep(0)

arm_command_publisher = rospy.Publisher('/arm_controller/command',
                                        JointTrajectory,
                                        queue_size=1)
target_point_publisher = rospy.Publisher('/arm_target',
                                         PointStamped,
                                         queue_size=1)

rospy.wait_for_service('/arm_controller/query_state')
query_trajectory_state = rospy.ServiceProxy('/arm_controller/query_state',
                                            QueryTrajectoryState)
joint_names = query_trajectory_state(rospy.Time.now()).name

robot_description = rospy.get_param('robot_description')  # type: str
ik = IK(robot_description, joint_names, 'arm_mount')
target = Target('arm_mount')
target.up(-math.pi / 6)
target.cw(math.pi)
target.retract(0.5)

pygame.init()
pygame.display.set_mode((100, 100))
pygame.display.set_caption('IK Controller')


def key_pressed(key):  # type: (str) -> bool
    index = pygame.__dict__['K_' + key]
    return pygame.key.get_pressed()[index]


rate = rospy.Rate(10)
while not rospy.is_shutdown():
    position = target.position(ik.frame())
    trajectory = ik.backward(position).to_command(rospy.Duration(0.01))
    arm_command_publisher.publish(trajectory)
    target_point_publisher.publish(target.point())

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            break
    if key_pressed('a'):
        target.ccw(0.05)
    if key_pressed('d'):
        target.cw(0.05)
    if key_pressed('w'):
        target.up(0.05)
    if key_pressed('s'):
        target.down(0.05)
    if key_pressed('e'):
        target.expand(0.01)
    if key_pressed('q'):
        target.retract(0.01)

    rate.sleep()
