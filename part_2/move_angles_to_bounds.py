import numpy as np

def move_angles_to_bounds(joint_angles_column_vector: np.ndarray) -> np.ndarray:
    new_angles = joint_angles_column_vector.copy()
    for i, angle in enumerate(new_angles[:, 0]):
        new_angle = angle
        if i != 4:
            while new_angle <= 0:
                new_angle += 360
            while new_angle >= 180:
                new_angle -= 360
        else:
            while new_angle <= 0:
                new_angle += 360
            while new_angle >= 270:
                new_angle -= 360

        new_angles[i, 0] = new_angle
    
    return new_angles