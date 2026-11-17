class Pid:
    def __init__(self, kp: float | int, kd: float | int, ki: float | int):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        
        self.prev_error: float | int | None = None
        self.integral = 0.0
    
    def control(self, measured_state: float | int, desired_state: float | int) -> float | int:
        # Calculate P, I, and D values for this measurement (given history)
        current_error = desired_state - measured_state
        if self.prev_error is None:
            current_error_derivative = 0
        else:
            current_error_derivative = (current_error - self.prev_error)
        self.integral += current_error
        current_error_integral = self.integral
        
        self.prev_error = current_error
        
        return self.kp * current_error + self.kd * current_error_derivative + self.ki * current_error_integral