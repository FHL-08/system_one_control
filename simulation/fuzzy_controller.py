"""Von-fuzzy speed controller over a simulated or serial plant.

Runs the shared controller (../shared/von_control.py — the same code the
Simulink hardware model calls) against:

  --simulate   SimMotor, the identified discrete-time plant (motor_sim.py)
  --port       the real motor via hardware/arduino/motor_firmware.ino

Usage:
  python fuzzy_controller.py --simulate --target 40
  python fuzzy_controller.py --port /dev/ttyUSB0 --target 40
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'shared'))
import von_control

TICK_S = 0.1   # loop period, matches von.cadence_s; inference is ~40 ms
TERMS = ['far_under', 'under', 'slightly_under', 'near_under', 'on_target',
         'near_over', 'slightly_over', 'over', 'far_over']


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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--simulate", action="store_true", help="use simulated motor")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--target", type=float,
                    default=von_control.PARAMS['ref_rpm'], help="target RPM")
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

    duty = 0.0
    t_start = time.time()

    print(f"target={args.target:g} RPM | {'SIM' if args.simulate else args.port}")
    print(f"{'t':>5} {'rpm':>7} {'err':>7} {'duty':>5}  memberships")

    try:
        for k in range(args.ticks):
            t0 = time.time()
            rpm = read_rpm()
            duty, mu, fired = von_control.tick(
                rpm, args.target, duty, t0 - t_start)
            set_duty(duty)

            mu_s = " ".join(f"{n}={v:.2f}" for n, v in zip(TERMS, mu))
            print(f"{k:>5} {rpm:7.0f} {rpm - args.target:+7.0f} {duty:5.2f}  {mu_s}")

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
