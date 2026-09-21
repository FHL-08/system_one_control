"""
Von-as-fuzzy-membership speed controller for a DC motor.

Idea: a fuzzy membership function is any map state -> [0,1] degree of truth.
Von's `judge`/Noul primitive returns a calibrated P(condition | state) in
[0,1] — i.e. a *learned, language-defined membership function*. We:

  1. Fuzzify the speed error into 5 linguistic terms, one Noul question each
     (all evaluated in ONE Von forward pass via system_one).
  2. Fire a Sugeno-0 rule base: each term's membership grade weights a
     constant duty-cycle adjustment.
  3. Defuzzify by weighted average and integrate onto the duty.

Usage:
  python fuzzy_controller.py --simulate --target 1500        # no hardware
  python fuzzy_controller.py --port /dev/ttyUSB0 --target 1500
"""

import argparse
import sys
import time
from collections import deque

import von
from von import Noul

# ---- fuzzy vocabulary: term -> (antecedent question, consequent duty delta) --
RULES = [
    ("far_under",     "Is the motor speed far below the target, less than roughly 70% of it?",        +0.20),
    ("under",         "Is the motor speed clearly below the target but not far below?",                +0.10),
    ("slightly_under","Is the motor speed slightly below the target?",                                 +0.04),
    ("near_under",    "Is the motor speed just barely below the target, within a few percent?",        +0.015),
    ("on_target",     "Is the motor speed approximately at the target?",                               0.000),
    ("near_over",     "Is the motor speed just barely above the target, within a few percent?",        -0.015),
    ("slightly_over", "Is the motor speed slightly above the target?",                                 -0.04),
    ("over",          "Is the motor speed clearly above the target but not far above?",                -0.10),
    ("far_over",      "Is the motor speed far above the target, more than roughly 140% of it?",        -0.20),
]

TICK_S = 0.5   # control period; Von on CPU takes ~0.1-1s per batch anyway


def fuzzify(state_text: str) -> dict:
    """One Von pass -> membership grade per fuzzy term."""
    resp = von.system_one(
        state=state_text,
        questions={
            name: Noul(instructions=q) for name, q, _ in RULES
        },
    )
    return {name: getattr(resp.answers[name], "noul") for name, _, _ in RULES}


def defuzzify(mu: dict) -> float:
    """Sugeno-0 weighted-average defuzzification -> duty delta in [-0.2, 0.2]."""
    num = sum(mu[name] * delta for name, _, delta in RULES)
    den = sum(mu[name] for name, _, _ in RULES)
    return num / den if den > 0 else 0.0


# ---- motor I/O --------------------------------------------------------------

class SerialMotor:
    def __init__(self, port: str, baud: int = 115200):
        import serial
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.rpm = 0.0
        time.sleep(2.0)  # wait for Arduino reset
        self.ser.reset_input_buffer()

    def set_duty(self, duty: float) -> None:
        self.ser.write(f"D{int(max(0.0, min(1.0, duty)) * 255)}\n".encode())

    def read_rpm(self) -> float:
        while self.ser.in_waiting:
            line = self.ser.readline().decode(errors="ignore").strip()
            if line.startswith("RPM "):
                try:
                    self.rpm = float(line[4:])
                except ValueError:
                    pass
        return self.rpm

    def close(self):
        self.ser.write(b"S\n")
        self.ser.close()


# ---- control loop -----------------------------------------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--simulate", action="store_true", help="use simulated motor")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--target", type=float, default=1500.0, help="target RPM")
    ap.add_argument("--backend", default="modernbert")
    ap.add_argument("--ticks", type=int, default=60)
    args = ap.parse_args()

    if args.simulate:
        from motor_sim import SimMotor
        motor = SimMotor()
        read_rpm = lambda: motor.step(TICK_S)
        set_duty = motor.set_duty
        close = lambda: None
    else:
        motor = SerialMotor(args.port)
        read_rpm = motor.read_rpm
        set_duty = motor.set_duty
        close = motor.close

    von.set_backend(args.backend)

    duty = 0.3  # feed-forward-ish starting point
    set_duty(duty)
    prev_err = 0.0
    history = deque(maxlen=5)

    print(f"target={args.target} RPM | {'SIM' if args.simulate else args.port}")
    print(f"{'t':>5} {'rpm':>7} {'err':>7} {'duty':>5}  memberships")

    try:
        for t in range(args.ticks):
            t0 = time.time()
            rpm = read_rpm()
            err = rpm - args.target
            trend = "increasing" if err > prev_err else "decreasing"
            prev_err = err

            state = (
                f"DC motor speed telemetry: target={args.target:.0f} RPM, "
                f"measured={rpm:.0f} RPM, error={err:+.0f} RPM, "
                f"error is {trend}."
            )

            mu = fuzzify(state)
            delta = defuzzify(mu)
            duty = max(0.0, min(1.0, duty + delta))
            set_duty(duty)

            mu_s = " ".join(f"{k}={v:.2f}" for k, v in mu.items())
            print(f"{t:>5} {rpm:7.0f} {err:+7.0f} {duty:5.2f}  {mu_s}")
            history.append((t, rpm, duty))

            dt = TICK_S - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)
    except KeyboardInterrupt:
        pass
    finally:
        set_duty(0.0)
        close()
        print("\nstopped (duty=0)")

    return 0


if __name__ == "__main__":
    sys.exit(main())
