"""Simulated DC motor: first-order lag + inertia + noise, standing in for the
Arduino+encoder so the demo runs without hardware.

rpm_dot = (duty * MAX_RPM - rpm) / TAU  +  load noise
"""

import random


class SimMotor:
    def __init__(self, max_rpm: float = 2000.0, tau_s: float = 0.8, noise_rpm: float = 8.0):
        self.max_rpm = max_rpm
        self.tau = tau_s
        self.noise = noise_rpm
        self.rpm = 0.0
        self.duty = 0.0  # 0..1

    def set_duty(self, duty: float) -> None:
        self.duty = max(0.0, min(1.0, duty))

    def step(self, dt_s: float) -> float:
        target = self.duty * self.max_rpm
        self.rpm += (target - self.rpm) * min(1.0, dt_s / self.tau)
        self.rpm += random.gauss(0.0, self.noise)
        self.rpm = max(0.0, self.rpm)
        return self.rpm
