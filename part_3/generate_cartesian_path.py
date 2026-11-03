import numpy as np
import numpy.typing as npt
from scipy.spatial.transform import Rotation, Slerp

def generate_cartesian_path(start_position: npt.NDArray[np.float64], end_position: npt.NDArray[np.float64], start_rotation: Rotation, end_rotation: Rotation, num_steps: int) -> tuple[list[npt.NDArray[np.float64]], list[Rotation]]:
    assert num_steps >= 2

    # Generate interpolation parameters from 0 to 1
    t = np.linspace(0.0, 1.0, num_steps)

    positions = [
        start_position + alpha * (end_position - start_position)
        for alpha in t
    ]

    key_rotations = Rotation.concatenate([start_rotation, end_rotation])
    slerp = Slerp([0.0, 1.0], key_rotations)
    
    rotations = list(slerp(t))
    
    return positions, rotations