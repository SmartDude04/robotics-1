import time

from Arm_Lib import Arm_Device
import numpy as np

class Dofbot:
    def __init__(self) -> None:
        self.arm = Arm_Device()
        time.sleep(0.25)
        
    def get_joint_angles(self) -> np.ndarray[tuple[int, int], np.dtype[np.float64]]:
        joints = np.zeros(shape=(6,1), dtype=np.float64)
        
        for i in range(1, 7):
            joints[i - 1] = self.arm.Arm_serial_servo_read(i)
        
        return joints
    
    def move_joint(self, joint_number: int, angle: np.float64, duration_milliseconds: float | int) -> None:
        self.arm.Arm_serial_servo_write(joint_number + 1, angle, duration_milliseconds)
        
    def move_joints(self, joints_array: np.ndarray[tuple[int, int], np.dtype[np.float64]], duration_milliseconds: float | int) -> None:
        self.arm.Arm_serial_servo_write6(joints_array[0, 0], joints_array[0, 1], joints_array[0, 2], joints_array[0, 3], joints_array[0, 4], joints_array[0, 5], duration_milliseconds)
    
    

d = Dofbot()
d.get_joint_angles()