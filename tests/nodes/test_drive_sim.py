#!/usr/bin/env python

import unittest
import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, GetModelStateResponse
import numpy as np
import transforms3d

from spear_station import pose_to_matrix


class TestDrive(unittest.TestCase):
    def setUp(self):
        rospy.init_node('test_drive')

        rospy.wait_for_service('/gazebo/get_model_state')
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state',
                                                  GetModelState)
        self.twist_publisher = rospy.Publisher('/drive_controller/cmd_vel',
                                               Twist)

    def _rover_pose(self):
        req = GetModelStateRequest()
        req.model_name = 'robot'
        resp = self.get_model_state(req)  # type: GetModelStateResponse
        return resp.pose

    def test_move_forward(self):
        initial_pose = self._rover_pose()

        # Move the rover forward for a bit
        for _ in range(10):
            t = Twist()
            t.linear.x = 1
            self.twist_publisher.publish(t)
            rospy.sleep(0.1)

        # Now stop
        for _ in range(10):
            self.twist_publisher.publish(Twist())
            rospy.sleep(0.1)

        final_pose = self._rover_pose()

        x = np.asarray([
            initial_pose.position.x, initial_pose.position.y,
            initial_pose.position.z
        ])
        y = np.asarray([
            final_pose.position.x, final_pose.position.y, final_pose.position.z
        ])
        distance = np.linalg.norm(x - y)
        self.assertAlmostEqual(distance, 1, delta=0.2)

    def test_turn(self):
        initial_pose = self._rover_pose()

        for _ in range(10):
            t = Twist()
            t.angular.z = 1
            self.twist_publisher.publish(t)
            rospy.sleep(0.1)

        for _ in range(10):
            self.twist_publisher.publish(Twist())
            rospy.sleep(0.1)

        final_pose = self._rover_pose()

        initial_mat = pose_to_matrix(initial_pose)
        final_mat = pose_to_matrix(final_pose)
        initial_to_final = np.matmul(final_mat, np.linalg.inv(initial_mat))
        _, R, _, _ = transforms3d.affines.decompose44(initial_to_final)
        _, angle = transforms3d.quaternions.quat2axangle(
            transforms3d.quaternions.mat2quat(R))
        self.assertAlmostEqual(abs(angle), 1, delta=0.2)


if __name__ == '__main__':
    import rostest
    rostest.rosrun('tests', 'test_drive', TestDrive)
