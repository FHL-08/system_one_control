"""Simulated DC motor using the identified discrete-time plant.

    x[k+1] = a*x[k] + b*volts[k],   Ts = 0.05 s

Constants come from shared/controller_params.json (same source the
Simulink model uses). Duty 0..1 maps onto 0..5 V via volts_per_duty.
step(dt_s) advances the 0.05 s model in substeps.
"""

import json
import random
from pathlib import Path

P = json.loads(
    (Path(__file__).resolve().parent.parent / 'shared' / 'controller_params.json').read_text())
TS = P['plant']['ts']
A = P['plant']['a']
B = P['plant']['b']
VOLTS = P['volts_per_duty']


class SimMotor:
    def __init__(self, noise_rpm: float = None):
        self.noise = P['noise_sigma_rpm'] if noise_rpm is None else noise_rpm
        self.rpm = 0.0
        self.duty = 0.0  # 0..1

    def set_duty(self, duty: float) -> None:
        self.duty = max(0.0, min(1.0, duty))

    def step(self, dt_s: float = 0.2) -> float:
        volts = self.duty * VOLTS
        for _ in range(max(1, round(dt_s / TS))):
            self.rpm = A * self.rpm + B * volts
            self.rpm += random.gauss(0.0, self.noise)
        self.rpm = max(0.0, self.rpm)
        return self.rpm
