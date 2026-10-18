import numpy as np
from scipy.spatial.transform import Rotation

position_weight = 1.0
rotation_weight = 0.01

def position_error(current_position, desired_position):
    """
    Used to find the error in position from the current and desired end effector position
    
    :param current_position: 3x1 vector representing the current end effector position
    :param desired_position: 3x1 vector representing the desired end effector position
    :return: 3x1 vector representing the distance between the current and desired end effector position
    """
    return position_weight * (desired_position - current_position)

def rotation_error(current_rotation, desired_rotation):
    """
    Used to find the error between the current and desired end effector rotations. Used Modified Rodrigues Parameters
    to find the axis and angle of rotation.
    
    :param current_rotation: 3x3 matrix representing the current end effector rotation
    :param desired_rotation: 3x3 matrix representing the desired end effector rotation
    :return: A 3x1 vector whose direction points in the axis of rotation and whose length (norm) is the
    degree difference
    """
    
    rotation_error_matrix = Rotation.from_matrix(desired_rotation.as_matrix() @ current_rotation.as_matrix().T)
    rotation_list = rotation_error_matrix.as_mrp()
    
    # Convert this list to a 2D column vector
    rotation_vector = np.atleast_2d(rotation_list).T
    if not rotation_vector.any():
        return rotation_vector
    
    # The returned magnitude is equal to tan(theta / 4), so convert to a magnitude of theta
    theta = np.arctan(np.linalg.norm(rotation_vector)) * 4
    rotation_vector = (rotation_vector / np.linalg.norm(rotation_vector)) * theta
    
    return rotation_weight * rotation_vector


def cost(position_error_vector, rotation_error_vector):
    # Scaling factor due to different units
    
    return np.linalg.norm(position_error_vector)**2 + np.linalg.norm(rotation_error_vector)**2