import time

from Arm_Lib import Arm_Device
import numpy as np

class Dofbot:
    def __init__(self):
        self.arm = Arm_Device()
        time.sleep(0.25)
        
    def get_joint_angles(self):
        joints = np.zeros(shape=(6,1), dtype=np.float64)
        
        for i in range(1, 7):
            print(self.arm.Arm_serial_servo_read(i), end="")
            joints[i - 1] = self.arm.Arm_serial_servo_read(i)
        
        return joints
    
    def move_joint(self, joint_number, angle, duration_milliseconds):
        self.arm.Arm_serial_servo_write(joint_number + 1, angle, duration_milliseconds)
        
    def move_joints(self, joints_array, duration_milliseconds):
        self.arm.Arm_serial_servo_write6(joints_array[0, 0], joints_array[0, 1], joints_array[0, 2], joints_array[0, 3], joints_array[0, 4], joints_array[0, 5], duration_milliseconds)