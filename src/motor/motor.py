from abc import ABC, abstractmethod

class Motor(ABC):
    def __init__(self, name, kp=1.0, ki=0.0, kd=0.0):
        self.name = name
        self.position = 0.0
        self.velocity = 0.0
        self.kp, self.ki, self.kd = kp, ki, kd
        self.prev_error = 0
        self.integral = 0

    @abstractmethod
    def apply_command(self, command):
        """Send command to the actual motor (e.g., PWM, CAN, etc.)"""
        pass

    def compute(self, setpoint, feedback, dt):
        """Internal PID controller."""
        error = setpoint - feedback
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

    def update(self, target_position, dt):
        command = self.compute(target_position, self.position, dt)
        self.apply_command(command)