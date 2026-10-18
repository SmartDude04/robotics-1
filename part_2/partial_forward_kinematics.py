import numpy as np
from scipy.spatial.transform import Rotation

# All positions, from P_01 to P_5T
positions_list = [
    np.atleast_2d([0, 0, 0.06100 + 0.04350]).T,
    np.zeros(shape=(3,1)),
    np.atleast_2d([0.08285, 0, 0]).T,
    np.atleast_2d([0, 0, -0.08285]).T,
    np.zeros(shape=(3,1)),
    np.atleast_2d([- (0.07385 + 0.05457), 0, 0]).T
]

def partial_forward_kinematics(joint_angles_column_vector, start_joint_number, end_joint_number):
    """
    Computes forward kinematics for the Dofbot arm up to a specific joint number. Used in inverse kinematics
    
    :param joint_angles_column_vector: 5x1 vector of joint angles, in degrees
    :param start_joint_number Starting joint number in the forward kinematics chain. From 0 to 6 (=T).
           Must be less than end_joint_number
    :param end_joint_number: Final joint number in the forward kinematics chain. From 0 to 6 (=T)
    :return: 3x1 position vector of the joint position and a Scipy rotation object
    """

    # All rotations, from R_01 to R_5T
    rotations_list = [
        Rotation.from_euler("z", joint_angles_column_vector[0, 0], degrees=True),
        Rotation.from_euler("y", -joint_angles_column_vector[1, 0], degrees=True),
        Rotation.from_euler("y", -joint_angles_column_vector[2, 0], degrees=True),
        Rotation.from_euler("y", -joint_angles_column_vector[3, 0], degrees=True),
        Rotation.from_euler("x", -joint_angles_column_vector[4, 0], degrees=True),
        Rotation.identity()
    ]
    
    if end_joint_number < start_joint_number:
        raise ValueError("start_joint_number must be less than or equal to end_joint_number")
    
    
    position_column_vector = np.zeros(shape=(3,1))
    for i in range(start_joint_number, end_joint_number):
        rotation_matrix = Rotation.identity()
        for j in range(start_joint_number, i):
            rotation_matrix *= rotations_list[j]
        
        position_column_vector += rotation_matrix.as_matrix() @ positions_list[i]
        
    rotation_matrix = Rotation.identity()
    for i in range(start_joint_number, end_joint_number):
        rotation_matrix *= rotations_list[i]
        
    return rotation_matrix, position_column_vector