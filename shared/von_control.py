"""Von fuzzy controller — single source of truth for the control law.

Constants live in controller_params.json (shared with the Simulink model
via load_params.m). This module is called from:
  - von_fuzzy.m      (MATLAB shim for motor_von_hw.slx, host-side)
  - von_vs_pid.py    (pure-Python offline comparison)

tick(rpm, target, duty_prev) -> (duty, mu): one controller step —
Noul posteriors as membership grades, Sugeno-0 weighted average,
error-magnitude throttle, deadband+leak, duty integrated and clamped
to [0,1].
"""
import json
from pathlib import Path

PARAMS = json.loads(
    (Path(__file__).parent / 'controller_params.json').read_text())
_noul = None
_e_prev = 0.0
_t_last = None
_mu_last = None
_mu_ema = None


def _ensure_ready():
    global _noul
    if _noul is None:
        import von
        von.set_backend('modernbert')
        from von_batch import noul_batch
        _noul = noul_batch


def reset():
    """Clear inter-call state (error trend, gate). Call between runs."""
    global _e_prev, _t_last, _mu_last, _mu_ema
    _e_prev = 0.0
    _t_last = None
    _mu_last = None
    _mu_ema = None


def tick(rpm, target, duty_prev, t):
    """One controller step at the model tick; inference gated by
    params.von.cadence_s — returns (duty, mu), holding both between
    inference calls."""
    _ensure_ready()
    global _e_prev, _t_last, _mu_last, _mu_ema
    v = PARAMS['von']

    # t < _t_last means a new run restarted the clock -> re-arm the gate
    if _t_last is not None and 0 <= t - _t_last < v['cadence_s']:
        return duty_prev, (_mu_last or [0.0] * 9), False
    _t_last = t

    err = rpm - target
    trend = 'increasing' if err > _e_prev else 'decreasing'
    _e_prev = err
    pct = 100 * err / target
    rel = ('at the target' if abs(pct) < 0.5 else
           f'{abs(pct):.0f}% below the target' if pct < 0 else
           f'{pct:.0f}% above the target')
    state = (f'DC motor speed telemetry: target={target:.0f} RPM, '
             f'measured={rpm:.0f} RPM ({rel}), '
             f'error={err:+.0f} RPM, error is {trend}.')

    mu = [float(g) for g in _noul(state, v['antecedents'])]
    # EMA on grades: a single noisy tick can't flip a saturated band
    beta = v.get('mu_ema', 0.0)
    if _mu_ema is None:
        _mu_ema = mu
    else:
        _mu_ema = [beta * p + (1 - beta) * m for p, m in zip(_mu_ema, mu)]
    mu = _mu_ema
    den = sum(mu)
    delta = (sum(m * c for m, c in zip(mu, v['consequents'])) / den
             if den > 0 else 0.0)

    # error-magnitude throttle: near target the biased under-grades would
    # otherwise keep pushing past the plant lag. Floored at
    # throttle_floor so corrections do not vanish in the last few RPM.
    # Braking gets its own (higher) floor: proportional, but with enough
    # authority at small overshoot to actually arrest the climb.
    floor = v['throttle_floor'] if delta > 0 else v['brake_floor']
    delta *= max(floor, min(1.0, abs(err) / v['err_fs']))

    # deadband: small integral leak inside +/-db_rpm
    if abs(err) <= v['db_rpm']:
        delta = -v['ki_leak'] * err

    # cadence invariance: consequents were tuned as duty-per-call at
    # 0.15 s; scale so the effective rate is cadence-independent
    delta *= v['cadence_s'] / 0.15

    _mu_last = mu
    return min(1.0, max(0.0, duty_prev + delta)), mu, True
