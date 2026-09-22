# Hardware

The plant is an **LGM12-N20 12mm DC geared motor** driven by an **Arduino
Uno** over PWM, with quadrature encoder feedback.

## Wiring (Arduino Uno)

| Pin | Function | Connects to |
|-----|----------|-------------|
| D2  | Encoder channel A (INT0) | Encoder output A |
| D3  | Encoder channel B (INT1) | Encoder output B |
| D5  | PWM output | Motor driver PWM/ENA input (L298N `ENA`, ESC signal, MOSFET gate driver) |

Provide your own driver stage and supply. Set `ENCODER_PPR` in the firmware
(or counts-per-rev on the Simulink Encoder block) to match your motor.

## Two ways to run

### Option A — Arduino firmware + Python controller

1. Flash `arduino/motor_firmware.ino` to the Uno.
2. From the repo root:

   ```bash
   .venv/bin/python simulation/fuzzy_controller.py --port /dev/ttyUSB0 --target 800
   ```

Serial protocol at 115200 baud: host sends `D<0-255>` (duty) or `S` (stop);
the board streams `RPM <float>` every 100 ms.

### Option B — Simulink Connected IO

`simulink/motor_von_hw.slx` runs the control loop host-side in real time;
PWM/Encoder blocks talk to the board over serial and the Von controller runs
through MATLAB's Python interface. Requires MATLAB, Simulink, and the
Simulink Support Package for Arduino Hardware.

See [simulink/README.md](simulink/README.md) for setup and run steps.
