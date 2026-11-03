import numpy as np
from scipy.spatial.transform import Rotation

from part_2.inverse_kinematics import inverse_kinematics
from part_3.generate_cartesian_path import generate_cartesian_path
from part_3.path_planning import path_planning


def main() -> None:
    path = generate_cartesian_path(np.asarray([0, 0, 0]), np.asarray([1, 0, 0]), Rotation.identity(), Rotation.from_euler("x", 90, degrees=True), 5)
    print(path)
    
    

if __name__ == "__main__":
    main()