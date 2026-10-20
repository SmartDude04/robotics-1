import numpy as np
from scipy.spatial.transform import Rotation

from part_1.forward_kinematics import forward_kinematics
from part_2.inverse_kinematics import inverse_kinematics


def main():
    current_joint_angles = np.atleast_2d([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T
    desired_pose = np.atleast_2d([0.23421554, 0.00817899, 0.10212526]).T
    desired_rotation = Rotation.from_euler("xyz", [1.57079633, -0.4712389, -3.10668607])
    
    new_joint_angles = inverse_kinematics(current_joint_angles, desired_pose, desired_rotation)
    print(new_joint_angles)
    print(forward_kinematics(new_joint_angles))
    
if __name__ == "__main__":
    main()