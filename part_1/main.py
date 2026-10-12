import time

from dofbot_comms import Dofbot
from forward_kinematics import forward_kinematics

def main() -> None:
    dofbot = Dofbot()
    
    while True:
        joint_angles = dofbot.get_joint_angles()
        print(joint_angles)
        
        # end_effector_rotation, end_effector_translation = forward_kinematics(joint_angles)
        # print(f"Rotation: {end_effector_rotation.as_matrix()}")
        # print(f"Translation: {end_effector_translation}")
        
        time.sleep(1.0)
        
    

if __name__ == "__main__":
    main()