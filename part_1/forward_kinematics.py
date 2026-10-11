import numpy as np
from scipy.spatial.transform import Rotation

def forward_kinematics(q: np.ndarray[tuple[int, int], np.dtype[np.float64]]) -> tuple[Rotation, np.ndarray[tuple[int, int], np.dtype[np.float64]]]:
    """
    Computes the end effector position and orientation relative to the base frame for Yahboom's Dofbot manipulator
    using the product of exponentials approach
    
    :param q: 5x1 vector of joint angles in degrees
    :return: The 3x3 rotation matrix describing the relative orientation of the end effector frame to the base frame
    (r_ {0t}) and the 3x1 vector describing the position of the end effector relative to the base, where the first
    element is the position along the base frame x-axis, the second element is the position along the base frame
    y-axis, and the third element is the position along the base frame z-axis (p_ {0t})
    """

    #set up the basis unit vectors
    e_x = np.array([1, 0, 0])
    e_y = np.array([0, 1, 0])
    e_z = np.array([0, 0, 1])

    # define the link lengths in meters
    l_0 = 0.06100 # base to servo 1
    l_1 = 0.04350 # servo 1 to servo 2
    l_2 = 0.08285 # servo 2 to servo 3
    l_3 = 0.08285 # servo 3 to servo 4
    l_4 = 0.07385 # servo 4 to servo 5
    l_5 = 0.05457 # servo 5 to gripper

    #set up the rotation matrices between subsequent frames
    r_01 = rotz(q[0]) # rotation between base frame and 1 frame
    r_12 = roty(-q[1]) # rotation between 1 and 2 frames
    r_23 = roty(-q[2]) # rotation between 2 and 3 frames
    r_34 = roty(-q[3]) # rotation between 3 and 4 frames
    r_45 = rotx(-q[4]) # rotation between 4 and 5 frames
    r_5t = roty(0.0) #the tool frame is defined to be the same as frame 5

    #set up the position vectors between subsequent frames
    p_01 = (l_0 + l_1) * e_z # translation between base frame and 1 frame in base frame
    p_12 = np.zeros(3,) # translation between 1 and 2 frame in 1 frame
    p_23 = l_2 * e_x # translation between 2 and 3 frame in 2 frame
    p_34 = -l_3 * e_z # translation between 3 and 4 frame in 3 frame
    p_45 = np.zeros(3,) # translation between 4 and 5 frame in 4 frame
    p_5t = -(l_4 + l_5) * e_x # translation between 5 and tool frame in 5 frame

    # calculate rot and pot
    #rot is a sequence of rotations
    r_0t = r_01 * r_12 * r_23 * r_34 * r_45 * r_5t
    #pot is a combination of the position vectors. 
    #    Each vector must be represented in the base frame before addition. 
    #    This is achieved using the rotation matrices.
    p_0t = p_01 + r_01.apply(p_12 + r_12.apply(p_23 + r_23.apply(p_34 + r_34.apply(p_45 + r_45.apply(p_5t)))))

    return r_0t, p_0t

def rotx(theta: int | float | np.float64) -> Rotation:
    """
    :param theta: Degrees to rotate
    :return: the principal axis rotation matrix for a rotation about the x-axis by theta degrees
    """
    
    r_x = Rotation.from_euler("x", theta, degrees=True)
    return r_x

def roty(theta: int | float | np.float64) -> Rotation:
    """
    :param theta: Degrees to rotate
    :return: the principal axis rotation matrix for a rotation about the y-axis by theta degrees
    """

    r_y = Rotation.from_euler("y", theta, degrees=True)
    return r_y

def rotz(theta: int | float | np.float64) -> Rotation:
    """
    :param theta: Degrees to rotate
    :return: the principal axis rotation matrix for a rotation about the z-axis by theta degrees
    """

    r_z = Rotation.from_euler("z", theta, degrees=True )
    return r_z