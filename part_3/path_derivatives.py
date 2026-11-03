import numpy as np
import numpy.typing as npt
from scipy.spatial.transform import Rotation, Slerp

def path_direction(first_position: npt.NDArray[np.float64], second_position: npt.NDArray[np.float64], fraction_of_path: np.float64) -> npt.NDArray[np.float64]:
    print(first_position)
    print(second_position)
    assert first_position.shape == second_position.shape == (3, 1)
    
    return (second_position - first_position) * fraction_of_path

def rotation_direction(first_rotation: Rotation, second_rotation: Rotation, fraction_of_path: np.float64) -> npt.NDArray[np.float64]:
    
    relative_rotation = second_rotation * first_rotation.inv()
    rotation_vector = np.atleast_2d(relative_rotation.as_rotvec()).T
    
    return rotation_vector * fraction_of_path