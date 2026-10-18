import numpy as np
from scipy.spatial.transform import Rotation

from part_1.forward_kinematics import forward_kinematics
from part_2.inverse_kinematics import inverse_kinematics


def main():
    current_joint_angles = np.atleast_2d([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T
    print(forward_kinematics(current_joint_angles))
    desired_pose = np.atleast_2d([0.1, 0, 0.02]).T
    desired_rotation = Rotation.identity()
    
    new_joint_angles = inverse_kinematics(current_joint_angles, desired_pose, desired_rotation)
    new_fk = forward_kinematics(new_joint_angles)
    print(new_fk[0].as_matrix(), new_fk[1])
    
if __name__ == "__main__":
    main()