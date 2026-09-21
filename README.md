# von-fuzzy-motor

**Fuzzy-logic motor speed control where the membership functions are a learned
decision model** — [Von](https://github.com/wfzyx/von), an open-source
"System One" model (bidirectional ModernBERT, ~400M params, Apache 2.0), plays
the role of the fuzzification layer. The plant is a DC motor driven by an
Arduino over PWM with encoder feedback.

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

### Rule base

| term            | antecedent (paraphrased)          | $c_i$  |
|-----------------|-----------------------------------|--------|
| far_under       | far below target ($< \sim 0.7\,\bar u$) | $+0.20$ |
| under           | clearly below target              | $+0.10$ |
| slightly_under  | slightly below target             | $+0.04$ |
| near_under      | barely below target               | $+0.015$ |
| on_target       | approximately at target           | $0$    |
| near_over       | barely above target               | $-0.015$ |
| slightly_over   | slightly above target             | $-0.04$ |
| over            | clearly above target              | $-0.10$ |
| far_over        | far above target ($> \sim 1.4\,\bar u$) | $-0.20$ |

## Repository layout

```
arduino/motor_firmware.ino    PWM drive + encoder tachometer + serial protocol
controller/fuzzy_controller.py  fuzzify → rule base → defuzzify → duty
controller/motor_sim.py       first-order-lag plant model for hardware-free runs
requirements.txt              Python deps (needs Python ≥ 3.12)
```

## Quickstart

```bash
uv venv --python 3.12 .venv
uv pip install --python .venv/bin/python -r requirements.txt

# simulated plant
.venv/bin/python controller/fuzzy_controller.py --simulate --target 1500

# real plant (flash arduino/motor_firmware.ino first)
.venv/bin/python controller/fuzzy_controller.py --port /dev/ttyUSB0 --target 1500
```

First run downloads ~1.5 GB of weights from Hugging Face (set `HF_TOKEN` for
higher rate limits). CPU inference is ~0.5–1 s per control tick.

## Hardware

Defaults in `motor_firmware.ino` — edit to match your setup:

| Arduino pin | Connects to |
|-------------|-------------|
| D9 (PWM)    | Driver PWM/enable (L298N `ENA`, ESC signal, MOSFET gate driver) |
| D2 (INT0)   | Encoder channel A, or single-channel tach (set `SINGLE_CHANNEL_TACH`) |
| D3 (INT1)   | Encoder channel B (quadrature only) |

Serial protocol at 115200 baud: host sends `D<0-255>` (duty) or `S` (stop);
board streams `RPM <float>` every 100 ms. Set `ENCODER_PPR` to your encoder.

## What a control theorist should know

**Simulated step response** (1500 RPM setpoint, first-order plant
$\dot\omega = (u\,\omega_{\max} - \omega)/\tau$, $\tau=0.8$ s): settles by
~5 s and holds within roughly ±3% with a mild limit cycle.

**The honest caveats:**

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
- **Rate limits.** ~1 Hz control loop on CPU. Suitable for supervisory
  loops, not inner-loop servo control.
- **Keep hard guardrails.** Out-of-distribution states return garbage grades;
  clamp duty and keep a hardware e-stop.

## License

MIT
