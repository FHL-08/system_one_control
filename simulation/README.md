# Simulation

Hardware-free runs of the control loop. Two entry points:

| Script | What it does |
|--------|--------------|
| `fuzzy_controller.py` | Self-contained controller demo. `--simulate` closes the loop around `motor_sim.py`; `--port` talks to the Arduino firmware instead. |
| `von_vs_pid.py` | Closed-loop PI vs Von comparison on the identified plant. Prints metrics; writes `comparison.png`, `von_memberships.png`, `comparison_result.npz`. |

The controller under test in `von_vs_pid.py` is `../shared/von_control.py` —
the same code the Simulink hardware model calls — with constants from
`../shared/controller_params.json`.

## Setup

From the repo root (one venv covers the whole project):

```bash
uv venv --python 3.12 .venv
uv pip install --python .venv/bin/python -r requirements.txt
```

The first Von call downloads ~1.5 GB of weights from Hugging Face (set
`HF_TOKEN` for higher rate limits). CPU inference is ~0.5–1 s per tick.

## Run

```bash
# quick demo — sim plant has DC gain ~1140 RPM at full duty, keep targets < ~1000
.venv/bin/python simulation/fuzzy_controller.py --simulate --target 800

# PI vs Von on the identified plant
.venv/bin/python simulation/von_vs_pid.py
```
