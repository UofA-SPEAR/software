from typing import Union

import numpy as np
import rospy
import transforms3d
from geometry_msgs.msg import Transform, TransformStamped
from tf2_ros import Buffer


def lookup_transform_simple(
        tf_buffer, target_frame,
        source_frame):  # type: (Buffer, str, str) -> TransformStamped
    while not rospy.is_shutdown():
        try:
            return tf_buffer.lookup_transform(target_frame=target_frame,
                                              source_frame=source_frame,
                                              time=rospy.Time(0),
                                              timeout=rospy.Duration(0.1))

        except Exception as err:
            rospy.logerr(err)


def transform_to_matrix(
        transform):  # type: (Union[Transform, TransformStamped]) -> np.ndarray
    # return msg_to_se3(transform)
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
