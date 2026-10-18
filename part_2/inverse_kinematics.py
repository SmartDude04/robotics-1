import numpy as np

from part_1.forward_kinematics import forward_kinematics
from part_2.errors import position_error, rotation_error, cost, position_weight, rotation_weight
from part_2.jacobian import jacobian

epsilon = 0.000001
step_amount = 0.2
inverse_damping = 0.01

max_iterations = 100

def inverse_kinematics(current_joint_angles_column_vector, desired_pose, desired_rotation):
    """
    Returns the joint angles needed to reach the desired pose and rotation
    :param current_joint_angles_column_vector: 5x1 vector of the current joint angles in degrees
    :param desired_pose: 3x1 vector of the desired end effector position in meters, in the base frame
    :param desired_rotation: Scipy rotation object for the desired end effector rotation in the base frame
    :return: A 5x1 vector of joint rotation angles, in degrees
    """
    current_rotation, current_pose = forward_kinematics(current_joint_angles_column_vector)
    current_pose = np.atleast_2d(current_pose).T
    
    current_position_error_vector = position_error(current_pose, desired_pose)
    current_rotation_error_vector = rotation_error(current_rotation, desired_rotation)
    
    iterative_joint_angles = np.copy(current_joint_angles_column_vector[0:5,0:1])
    
    # Keep iterating until the total error is less than the desired amount
    i = 0
    while cost(current_position_error_vector, current_rotation_error_vector) > epsilon and i < max_iterations:
        jacobian_matrix = jacobian(iterative_joint_angles)
        
        # Multiply the rows by the weights from errors.py
        jacobian_matrix[0:3, 0:5] *= rotation_weight
        jacobian_matrix[3:6, 0:5] *= position_weight
        
        # Calculate the Moore-Penrose pseudo-inverse of the Jacobian
        jacobian_inverse = jacobian_matrix.T @ np.linalg.inv(jacobian_matrix @ jacobian_matrix.T + inverse_damping**2 * np.identity(6))
        
        
        # Now we can input desired (spatial) velocity and get desired angular rates
        # Alpha is used as a unit scale (has units time^(-1))
        desired_spatial_velocity = np.zeros(shape=(6,1))
        desired_spatial_velocity[0:3,0:1] = current_rotation_error_vector
        desired_spatial_velocity[3:6,0:1] = current_position_error_vector
        
        desired_joint_angular_rates = jacobian_inverse @ desired_spatial_velocity
        
        # Now add that to the current joint angle list and repeat
        iterative_joint_angles += (step_amount * (180.0 / np.pi) * desired_joint_angular_rates)
        
        # Update the error vectors
        current_rotation, current_pose = forward_kinematics(iterative_joint_angles)
        current_pose = np.atleast_2d(current_pose).T
        current_position_error_vector = position_error(current_pose, desired_pose)
        current_rotation_error_vector = rotation_error(current_rotation, desired_rotation)
        
        i += 1
        
    return iterative_joint_angles