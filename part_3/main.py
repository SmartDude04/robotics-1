import numpy as np
from scipy.spatial.transform import Rotation

from part_1.dofbot_comms import Dofbot
from part_2.inverse_kinematics import inverse_kinematics
from part_3.generate_cartesian_path import generate_cartesian_path
from part_3.path_planning import path_planning


def main() -> None:
    dofbot = Dofbot()
    move_end_effector_poses = np.zeros(shape=(3,5,2))

    move_end_effector_poses[:, 0, 0] = [0.23273228, 0.00812719, 0.09465976]
    move_end_effector_poses[:, 0, 1] = [1.57079633, -0.50614548, -3.10668607]
    move_end_effector_poses[:, 1, 0] = [0.19894388, 0.18551817, 0.16422239]
    move_end_effector_poses[:, 1, 1] = [1.57079633, 0.2443461, -2.39110108]
    move_end_effector_poses[:, 2, 0] = [0.12832051, 0.17028706, 0.28247915]
    move_end_effector_poses[:, 2, 1] = [1.57079633, 0.61086524, -2.21656815]
    move_end_effector_poses[:, 3, 0] = [0.14949413, 0.22163416, 0.08686313]
    move_end_effector_poses[:, 3, 1] = [1.04719755, -0.15707963, -2.16420827]
    move_end_effector_poses[:, 4, 0] = [0.11713209, 0.13474506, 0.09047552]
    move_end_effector_poses[:, 4, 1] = [1.04719755, -0.95993109, -2.28638132]
    
    for i in range(2):
        cur_position = move_end_effector_poses[:, i, 0:1]
        cur_rotation = Rotation.from_euler("xyz", move_end_effector_poses[:, i, 1])
        next_position = move_end_effector_poses[:, i + 1, 0:1]
        next_rotation = Rotation.from_euler("xyz", move_end_effector_poses[:, i + 1, 1])
        
        current_joint_angles = dofbot.get_joint_angles()
        desired_joint_angles = inverse_kinematics(current_joint_angles, cur_position, cur_rotation)
        desired_joint_angles = np.vstack((desired_joint_angles, [[90.0]]))
        dofbot.move_joints(desired_joint_angles, 1000)
        
        input("Enter when ready to move along the path")
        
        position_path, rotation_path = generate_cartesian_path(cur_position, next_position, cur_rotation, next_rotation, 20)
        joint_space_path = path_planning(dofbot.get_joint_angles()[:5,:], position_path, rotation_path)
        
        for joint_angles in joint_space_path:
            dofbot.move_joints(joint_angles, 100)
        

if __name__ == "__main__":
    main()