import numpy as np
from scipy.spatial.transform import Rotation

from part_2.partial_forward_kinematics import partial_forward_kinematics

# List of axes of rotation for each joint, h_1 through h_5
axis_of_rotation = [
    np.atleast_2d([0, 0, 1]).T,
    np.atleast_2d([0, -1, 0]).T,
    np.atleast_2d([0, -1, 0]).T,
    np.atleast_2d([0, -1, 0]).T,
    np.atleast_2d([-1, 0, 0]).T,
]

def jacobian(joint_angles):
    """
    Computes the jacobian for the Dofbot given a column vector of joint angles
    :param joint_angles: 5x1 vector of joint angles in degrees
    :return: The jacobian matrix for the Dofbot
    """
    
    jacobian = np.zeros(shape=(6,5))
    
    for column in range(5):
        rotation_0_i = partial_forward_kinematics(joint_angles, 0, column + 1)[0].as_matrix()
        position_i_t = partial_forward_kinematics(joint_angles, column + 1, 6)[1]
        
        # Set the first three rows as R_0,i @ h_i
        jacobian[0:3, column:column+1] = rotation_0_i @ axis_of_rotation[column]
        
        # Set the last three rows as (R_0, i @ h_i) x (R_0, i + 1 @ P_i + 1, T)
        jacobian[3:6, column] = np.cross(jacobian[0:3, column], (rotation_0_i @ position_i_t).ravel())
    
    return jacobian