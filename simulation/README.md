# Simulation

Hardware-free runs of the control loop. Two entry points:

| Script | What it does |
|--------|--------------|
| `fuzzy_controller.py` | Closes the loop with the shared Von controller. `--simulate` runs it against `motor_sim.py` (the identified plant); `--port` drives the real motor over serial. |
| `von_vs_pid.py` | Closed-loop PI vs Von comparison on the identified plant. Prints metrics; writes `comparison.png`, `von_memberships.png`, `comparison_result.npz`. |
| `membership_sweep.py` | Sweeps measured RPM across the band edges and audits membership coherence for a premise/antecedent wording. Prints per-term band separation; writes `membership_sweep.png`, `membership_sweep.npz`. |
| `tune_consequents.py` | Closed-loop grid search over consequents / throttle floors / grade EMA using the `von_vs_pid` harness. |

All four use `../shared/von_control.py` — the same code the Simulink
hardware model calls — with constants from `../shared/controller_params.json`.

## Setup

From the repo root (one venv covers the whole project):

```bash
uv venv --python 3.12 .venv
uv pip install --python .venv/bin/python -r requirements.txt
```

The first Von call downloads ~1.5 GB of weights from Hugging Face (set
`HF_TOKEN` for higher rate limits). Inference is ~40 ms per tick on a
desktop CPU; updates are gated by `von.cadence_s` (10 Hz).

## Run

```bash
# default target is ref_rpm from shared/controller_params.json (40 RPM);
# the identified plant tops out around 56 RPM at full duty
.venv/bin/python simulation/fuzzy_controller.py --simulate --target 40

# PI vs Von on the identified plant
.venv/bin/python simulation/von_vs_pid.py
```
