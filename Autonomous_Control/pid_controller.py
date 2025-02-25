from dataclasses import dataclass

@dataclass
class PIDParams:
    kp: float
    ki: float
    kd: float

# PID Controller class
class PIDController:
    def __init__(self, params: PIDParams):
        self.params = params
        self.max_output = 16    # max steering angle degrees
        self.prev_error = 0
        self.integral = 0

    def update(self, error: float, dt: float) -> float:
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = (self.params.kp * error + self.params.ki * self.integral + self.params.kd * derivative)
        self.prev_error = error
        return output