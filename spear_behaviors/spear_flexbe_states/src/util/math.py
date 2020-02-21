import numpy as np
from geometry_msgs.msg import Quaternion
from tf import transformations
from ros_numpy import numpify


def quaternion_to_yaw(quaternion):  # type: (Quaternion) -> float
    yaw, _, _ = transformations.rotation_from_matrix(transformations.quaternion_matrix(numpify(quaternion)))
    return yaw


def angular_diff(angle1, angle2):  # type: (float, float) -> float
    """
    Returns the smallest angular difference between two angles

    Args:
        angle1: First angle, in radians
        angle2: Second angle, in radians

    Returns:
        Angular difference in radians between the two angles, in the range [-pi, pi]
    """
    diff = angle1 - angle2
    while diff < -np.pi:
        diff += 2*np.pi
    while diff > np.pi:
        diff -= 2*np.pi
    return diff


def quaternion_rotate(quaternion, vector):  # type: (np.ndarray, np.ndarray) -> np.ndarray
    q = quaternion
    v = np.concatenate((vector, [0]))
    qc = transformations.quaternion_conjugate(quaternion)
    return transformations.quaternion_multiply(transformations.quaternion_multiply(q, v), qc)


def normal_to_quaternion(normal):  # type: (np.ndarray) -> np.ndarray
    rotation_matrix = np.zeros((4,4))
    rotation_matrix[3,3] = 1.0
    rotation_matrix[:3,:3] = np.vstack((
        normal,
        np.cross(np.array([0.0, 0.0, 1.0]), normal),
        np.array([0.0, 0.0, 1.0]),
    )).T
    return transformations.quaternion_from_matrix(rotation_matrix)
