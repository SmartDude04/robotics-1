import time

import numpy as np

from dofbot_comms import Dofbot
from forward_kinematics import forward_kinematics

def main() -> None:
    dofbot = Dofbot()
    
    move_joint_angles = np.zeros(shape=(6, 1))
    move_joint_angles[:, 0] = [90, 90, 90, 90, 90, 90]
    dofbot.move_joints(move_joint_angles, 1000)
    
    while True:
        joint_angles = dofbot.get_joint_angles()
        print(joint_angles)
        
        # end_effector_rotation, end_effector_translation = forward_kinematics(joint_angles)
        # print(f"Rotation: {end_effector_rotation.as_matrix()}")
        # print(f"Translation: {end_effector_translation}")
        
        time.sleep(1.0)
        
    

if __name__ == "__main__":
    main()