# von-fuzzy-motor

**Fuzzy-logic motor speed control where the membership functions are a learned
decision model** — [Von](https://github.com/wfzyx/von), an open-source
"System One" model (bidirectional ModernBERT, ~400M params, Apache 2.0), plays
the role of the fuzzification layer. The plant is an LGM12-N20 12mm DC geared
motor driven by an Arduino Uno over PWM with encoder feedback.

## Motivation

A fuzzy controller needs a fuzzification map

$$\mu_i : \mathcal{X} \to [0,1]$$

assigning each plant state $x$ a degree of membership in linguistic term $i$.
Conventionally $\mu_i$ are hand-shaped triangles, trapezoids, or Gaussians.
This project replaces them with Von's binary-verification primitive ("Noul"),
which returns a calibrated posterior

$$\mu_i(x) = P\bigl(\text{term}_i \text{ holds} \,\big|\, x\bigr) \in [0,1]$$

where each term is defined in natural language ("is the speed slightly above
the target?"). Nothing in the fuzzy inference chain requires $\mu$ to be
analytic — any state-graded map in $[0,1]$ is a valid membership function —
so the swap is mathematically sound and gives language-programmable,
calibrated grades for free. In spirit this is a zero-shot ANFIS: the
membership shapes were learned during the model's entailment-style training
rather than tuned per-plant.

## Control law

Each tick the controller serializes telemetry into text,
$x = (\text{target},\ \text{rpm},\ e,\ \dot e\ \text{sign})$, and evaluates all
$N$ antecedents in a single model forward pass. Rule consequents are
singletons $c_i$ (Sugeno order-0), defuzzified by weighted average and
integrated onto the duty cycle:

$$\mu_i = \text{Noul}\bigl(x,\ q_i\bigr), \qquad
u \leftarrow \Pi_{[0,1]}\!\left[u + \frac{\sum_i \mu_i c_i}{\sum_i \mu_i}\right]$$

The implementation adds an error-magnitude throttle (scales the delta by
$|e|/e_{fs}$), a deadband with a small integral leak, and cadence
normalization — all constants in `shared/controller_params.json`.

### Rule base

| term            | antecedent (paraphrased)          | $c_i$  |
|-----------------|-----------------------------------|--------|
| far_under       | far below target ($< \sim 0.7\,\bar u$) | $+0.25$ |
| under           | clearly below target              | $+0.03$ |
| slightly_under  | slightly below target             | $+0.002$ |
| near_under      | barely below target               | $+0.001$ |
| on_target       | approximately at target           | $0$    |
| near_over       | barely above target               | $-0.01$ |
| slightly_over   | slightly above target             | $-0.04$ |
| over            | clearly above target              | $-0.15$ |
| far_over        | far above target ($> \sim 1.4\,\bar u$) | $-0.2$ |

## Repository layout

```
shared/                   Von fuzzy controller used by both paths below
  von_control.py            one control tick: fuzzify -> rule base -> duty
  von_batch.py              evaluates all antecedents in one forward pass
  controller_params.json    plant, PI and Von constants (single source of truth)
simulation/               hardware-free runs — see simulation/README.md
hardware/
  arduino/                  standalone Uno firmware — see hardware/README.md
  simulink/                 Connected IO models + MATLAB scripts — see
                            hardware/simulink/README.md
requirements.txt          Python deps (needs Python >= 3.12)
```

## Environment setup

One Python environment covers both the simulation and the Simulink hardware
path (MATLAB calls it via `pyenv`):

```bash
uv venv --python 3.12 .venv
uv pip install --python .venv/bin/python -r requirements.txt
```

First Von call downloads ~1.5 GB of weights from Hugging Face (set `HF_TOKEN`
for higher rate limits). Inference is ~40 ms per control tick on a desktop
CPU — all 9 antecedents in a single forward pass.

## What to run

- **Simulation (no hardware):** see [simulation/README.md](simulation/README.md) —
  `fuzzy_controller.py --simulate` for a quick demo, `von_vs_pid.py` for the
  PI-vs-Von comparison.
- **Hardware:** see [hardware/README.md](hardware/README.md) — either the
  standalone Arduino firmware driven over serial, or the Simulink
  Connected IO model in [hardware/simulink/](hardware/simulink/README.md).

## Technical notes

**Simulated step response.** `simulation/von_vs_pid.py` uses the identified
discrete-time plant from `shared/controller_params.json`:

$$G(z) = \frac{0.7826\,z^{-1}}{1 - 0.9300\,z^{-1}}, \qquad T_s = 0.05\text{ s}$$

with duty $u \in [0,1]$ mapped onto the 0–5 V input
(`volts = volts_per_duty * duty`). DC gain ≈ 11.2 RPM/V, so the output shaft
tops out around 56 RPM at full duty; the reference is 40 RPM.

**Limitations:**

- **The membership map is not analytic.** Grades are a learned lookup over
  serialized text, so there are no continuity/monotonicity guarantees and no
  classical (Lyapunov/describing-function) stability proof. It is an empirical
  controller.
- **Grades are crisp-ish.** Calibrated probabilities cluster near 0/1, so the
  effective surface is closer to a coarse switching surface than a smooth
  fuzzy one; the fine "near"/"slightly" terms do the steady-state work.
- **Asymmetry is real and exploitable.** "Under" terms fire more readily than
  mirrored "over" terms (lexical entailment bias from NLI-style training).
  The result is an asymmetric gain surface — brakes harder than it
  accelerates — which is *desirable* when overshoot/overcurrent is the
  dangerous direction, and can be engineered deliberately via wording.
- **Rate limits.** Control updates are gated by `von.cadence_s`
  (0.1 s → 10 Hz) in `shared/controller_params.json`; inference itself is
  ~40 ms/batch on CPU.
- **Keep hard guardrails.** Out-of-distribution states return garbage grades;
  clamp duty and keep a hardware e-stop.

## License

MIT
