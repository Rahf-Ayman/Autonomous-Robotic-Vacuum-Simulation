"""Bounded PID controller with anti-windup and first-step derivative guard."""

from config import MAX_SPEED, clamp


class PIDController:
    """Bounded proportional-integral-derivative controller with anti-windup."""

    def __init__(
        self, kp, ki, kd, setpoint=0.0, output_max=MAX_SPEED, output_min=-MAX_SPEED
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.previous_error = 0.0
        self.integral = 0.0
        self.first_run = True
        self.output_max = output_max
        self.output_min = output_min

    def compute(self, process_value, timestep_s):
        """Return bounded control output from a process value."""
        return self.compute_error(self.setpoint - process_value, timestep_s)

    def compute_error(self, error, timestep_s):
        """Return bounded control output directly from a pre-computed signed error."""
        if timestep_s <= 0.0:
            return 0.0

        proportional = self.kp * error

        # Update integral with anti-windup clamping
        self.integral += error * timestep_s
        if self.ki > 0.0:
            max_i = self.output_max / self.ki
            min_i = self.output_min / self.ki
            self.integral = clamp(self.integral, min_i, max_i)
        integral = self.ki * self.integral

        # Avoid derivative spike on the very first step after reset
        if self.first_run:
            derivative = 0.0
            self.first_run = False
        else:
            derivative = self.kd * ((error - self.previous_error) / timestep_s)
        self.previous_error = error

        return clamp(
            proportional + integral + derivative,
            self.output_min,
            self.output_max,
        )

    def reset(self, setpoint=None):
        """Clear accumulated state and optionally update the setpoint."""
        self.integral = 0.0
        self.previous_error = 0.0
        self.first_run = True
        if setpoint is not None:
            self.setpoint = setpoint