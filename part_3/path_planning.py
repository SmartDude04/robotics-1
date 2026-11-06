import numpy as np
import numpy.typing as npt
from scipy.spatial.transform import Rotation
from scipy import sparse
import osqp

from part_1.forward_kinematics import forward_kinematics
from part_2.jacobian import jacobian
from part_3.path_derivatives import path_direction, rotation_direction

damping = 1e-4
q_min = np.array([0, 0, 0, 0, 0], dtype=np.float64)
q_max = np.array([180, 180, 180, 180, 270], dtype=np.float64)
dq_max = np.full(5, 2.0)


def path_planning(initial_joint_configuration: npt.NDArray[np.float64], position_path: list[npt.NDArray[np.float64]], rotation_path: list[Rotation]) -> list[npt.NDArray[np.float64]]:
    """
    Given a 3xN position path and N-dimensional list of Rotation objects, generates a joint-space path for the Dofbot
    
    :param initial_joint_configuration 
    :param position_path: 3xN position path the Dofbot should follow in task space
    :param rotation_path: N-dimensional list of End Effector rotation objects the Dofbot should follow in task space
    :return: 5xN path the Dofbot should follow in joint space
    """
    
    for position in position_path:
        assert position.shape == (3, 1)
    assert initial_joint_configuration.shape == (5, 1)
    for num in initial_joint_configuration:
        assert num != np.nan
    assert len(position_path) == len(rotation_path)
        
    joint_space_path = [np.copy(initial_joint_configuration)]
    
    # OSQP Solver:
    # Minimize: 1/2 x^T P x + q^T x
    # Subject to: l <= A x <= u
    
    
    A = sparse.eye(5, format="csc")
    
    for i in range(len(position_path) - 1):
        current_rotation, current_position = forward_kinematics(joint_space_path[-1])
        current_position = np.atleast_2d(current_position).T
        target_position, target_rotation = np.copy(position_path[i + 1]), rotation_path[i + 1]
        
        position_derivative = path_direction(current_position, target_position, np.float64(1.0))
        rotation_derivative = rotation_direction(current_rotation, target_rotation, np.float64(1.0))
        
        J = jacobian(joint_space_path[-1]) * (np.pi / 180.0)
        
        H = 2 * (J.T @ J + damping * np.eye(5))
        P = sparse.triu(
            sparse.csc_matrix(H),
            format="csc"
        )
        q = -2 * J.T @ np.vstack((rotation_derivative, position_derivative))

        angles = joint_space_path[-1].ravel()
        lower_bound = np.maximum(q_min - angles, -dq_max)
        upper_bound = np.minimum(q_max - angles, dq_max)
        
        solver = osqp.OSQP()
        
        solver.setup(
            P=P,
            q=q.flatten(),
            A=A,
            l=lower_bound,
            u=upper_bound,
            verbose=False
        )
        
        result = solver.solve()
        if result.info.status not in ("solved", "solved inaccurate"):
            raise RuntimeError(
                f"QP failed at waypoint {i}: "
                f"{result.info.status}"
            )
        
        joint_space_delta = result.x.reshape(5, 1)
        next_joint_space_vector = joint_space_path[-1] + joint_space_delta
        joint_space_path.append(next_joint_space_vector)
    
    # Add EE grasp amount
    for i in range(len(joint_space_path)):
        joint_space_path[i] = np.vstack((joint_space_path[i], [[90.0]]))
    
    return joint_space_path
        