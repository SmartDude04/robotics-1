import time

import numpy as np

from part_1.dofbot_comms import Dofbot
from part_1.forward_kinematics import forward_kinematics
from part_2.inverse_kinematics import inverse_kinematics


def main() -> None:
    dofbot = Dofbot()
    
    move_joint_angles = np.zeros(shape=(6, 5))
    move_joint_angles[:, 0] = [2, 62, 16, 75, 90, 90]
    move_joint_angles[:, 1] = [44, 37, 41, 118, 90, 90]
    move_joint_angles[:, 2] = [54, 70, 39, 105, 90, 90]
    move_joint_angles[:, 3] = [57, 34, 26, 112, 121, 90]
    move_joint_angles[:, 4] = [49, 75, 23, 27, 121, 90]
    
    for i in range(move_joint_angles.shape[1]):
        dofbot.move_joints(move_joint_angles[:, i:i + 1], 1000)
        time.sleep(0.5)
        
        measured_joint_angles_column_vector = dofbot.get_joint_angles()
        rotation_matrix, translation_column_vector = forward_kinematics(measured_joint_angles_column_vector)
        
        print("Rotation matrix:")
        print(rotation_matrix.as_matrix())
        
        print("\nTranslation column vector:")
        print(translation_column_vector)
        
        ik_joint_angles = inverse_kinematics(np.atleast_2d([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T, np.atleast_2d(translation_column_vector).T, rotation_matrix)
        
        input("Move to IK solution")
        ik_joint_angles = np.vstack((ik_joint_angles, [[90.0]]))
        dofbot.move_joints(ik_joint_angles, 1000)
        
        if i != move_joint_angles.shape[1] - 1:
            input("\nPress Enter when ready to move to the next point")
    
    print("Finished...")
    

if __name__ == "__main__":
    main()