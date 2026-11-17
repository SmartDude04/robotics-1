class Pid:
    def __init__(self, kp: float | int, kd: float | int, ki: float | int, timestep_seconds: float | int):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.timestep_seconds = timestep_seconds
        
        # Stores the previous errors; initialize with a single value for continuity between all function calls
        self.error_history: list[float | int] = []
        self.integral = 0.0
    
    def control(self, measured_state: float | int, desired_state: float | int) -> float | int:
        # Calculate P, I, and D values for this measurement (given history)
        current_error = desired_state - measured_state
        if len(self.error_history) == 0:
            current_error_derivative = 0.0
        else:
            current_error_derivative = (current_error - self.error_history[-1]) / self.timestep_seconds
        self.integral += current_error * self.timestep_seconds
        current_error_integral = self.integral
        
        # Add the value to the history
        self.error_history.append(current_error)
        
        return self.kp * current_error + self.kd * current_error_derivative + self.ki * current_error_integral