from typing import Optional, TypeVar, Union

import numpy as np
import rospy
import tf2_geometry_msgs
import transforms3d
from geometry_msgs.msg import (PointStamped, Pose, PoseStamped, Transform,
                               TransformStamped, Vector3Stamped, WrenchStamped)
from tf2_ros import Buffer

from .util import get_tf_buffer


def lookup_transform_simple(
        target_frame,
        source_frame,
        tf_buffer=None
):  # type: (str, str, Optional[Buffer]) -> TransformStamped
    if tf_buffer is None:
        tf_buffer = get_tf_buffer()
    while not rospy.is_shutdown():
        try:
            return tf_buffer.lookup_transform(target_frame=target_frame,
                                              source_frame=source_frame,
                                              time=rospy.Time(0),
                                              timeout=rospy.Duration(0.1))

        except Exception as err:
            rospy.logerr(err)


TransformableMessage = TypeVar("TransformableMessage", PointStamped,
                               Vector3Stamped, WrenchStamped, PoseStamped)


def do_transform_msg(msg, target_frame, transform=None):
    if transform is None:
        transform = lookup_transform_simple(target_frame, msg.header.frame_id)
    if isinstance(msg, PointStamped):
        return tf2_geometry_msgs.do_transform_point(msg, transform)
    if isinstance(msg, Vector3Stamped):
        return tf2_geometry_msgs.do_transform_vector3(msg, transform)
    if isinstance(msg, WrenchStamped):
        return tf2_geometry_msgs.do_transform_wrench(msg, transform)
    if isinstance(msg, PoseStamped):
        return tf2_geometry_msgs.do_transform_pose(msg, transform)
    raise TypeError("Unexpected message type {}".format(type(msg)))


def transform_to_matrix(
        transform):  # type: (Union[Transform, TransformStamped]) -> np.ndarray
    """
    Converts a Transform message to the corresponding homogeneous transformation
    matrix (an np array).
    """
    if isinstance(transform, TransformStamped):
        transform = transform.transform

    translation = [
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
    ]
    # w is first (unlike standard ROS), because that's what quat2mat expects
    rotation = [
        transform.rotation.w,
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
    ]
    return transforms3d.affines.compose(
        T=translation,
        R=transforms3d.quaternions.quat2mat(rotation),
        Z=np.ones(shape=(3, )))

def pose_to_matrix(pose):  # type: (Union[Pose, PoseStamped]) -> np.ndarray
    if isinstance(pose, PoseStamped):
        pose = pose.pose

    transform = Transform()
    transform.translation.x = pose.position.x
    transform.translation.y = pose.position.y
    transform.translation.z = pose.position.z
    transform.rotation = pose.orientation
    return transform_to_matrix(transform)
