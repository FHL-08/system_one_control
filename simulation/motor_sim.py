"""Simulated DC motor using the identified discrete-time model

    G(z) = 172 / (z - 0.2462),   Ts = 0.2 s,  input = 0..5 V

i.e.  omega[k+1] = 0.2462 * omega[k] + 172 * volts[k]
Duty 0..1 maps linearly onto the 0..5 V input range.
"""

import random


class SimMotor:
    def __init__(self, v_max: float = 5.0, noise_rpm: float = 4.0):
        self.v_max = v_max
        self.noise = noise_rpm
        self.rpm = 0.0
        self.duty = 0.0  # 0..1

    def set_duty(self, duty: float) -> None:
        self.duty = max(0.0, min(1.0, duty))

    def step(self, dt_s: float = 0.2) -> float:
        volts = self.duty * self.v_max
        self.rpm = 0.2462 * self.rpm + 172.0 * volts
        self.rpm += random.gauss(0.0, self.noise)
        self.rpm = max(0.0, self.rpm)
        return self.rpm
