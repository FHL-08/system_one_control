# Simulink (Connected IO)

Closed-loop control of the LGM12-N20 from MATLAB. The model runs host-side
in real time while the PWM and Encoder blocks communicate with the Uno over
serial; `von_fuzzy.m` calls `../../shared/von_control.py` through `pyenv`.

## Requirements

- MATLAB + Simulink
- Simulink Support Package for Arduino Hardware (Connected IO uploads the
  IO server to the board automatically on first run)
- The project Python venv: set it up from the repo root first; `setup.m`
  points `pyenv` at `.venv/bin/python`

## Files

| File | Role |
|------|------|
| `generate_identification_data.slx` | Applies the input sequence and logs `t`, `u`, `y` for identification |
| `motor_data.mat` | Captured identification dataset (`t`, `u`, `y`); lets you run the ID script without hardware |
| `optimal_motor_identification.m` | Fits a first-order IIR model to `t`, `u`, `y` (expects them in the workspace) |
| `design_pid.m` | Pole-placement PI design; writes gains to `shared/controller_params.json` |
| `motor_von_hw.slx` | Closed-loop model; CtrlSelect switches Von (in1) vs PI (in2) |
| `load_params.m` | Reads `shared/controller_params.json` (model InitFcn) |
| `encoder_rpm.m` | Encoder count to RPM using wall-clock dt |
| `von_fuzzy.m` | MATLAB shim around `von_control.tick` |
| `setup.m` | One-time per session: venv interpreter, Connected IO mode, Von warmup |
| `run_hardware.m` | Timed closed-loop run; detects the CtrlSelect position and saves `hw_von.mat`/`.png` or `hw_pi.mat`/`.png` (overwritten each run) |

## Run

From this folder in MATLAB:

```matlab
>> setup                  % once per MATLAB session
>> out = run_hardware(30);  % 30 s run -> hw_von.mat or hw_pi.mat (+ .png), per CtrlSelect
```

To re-identify the plant or retune the PI:

```matlab
>> load motor_data        % or run generate_identification_data.slx for fresh t,u,y
>> optimal_motor_identification
>> design_pid
```

