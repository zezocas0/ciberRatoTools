
from datetime import datetime

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error_sum = 0
        self.last_error = 0
        self.last_time = datetime.now()

    def update(self, error):
        # Calculate the time step as the difference between the current time and the last time
        time_now = datetime.now()
        dt = (time_now - self.last_time).total_seconds()
        self.last_time = time_now


        self.error_sum += error * dt
        derivative = (error - self.last_error) / dt
        self.last_error = error
        return self.kp * error + self.ki * self.error_sum + self.kd * derivative


