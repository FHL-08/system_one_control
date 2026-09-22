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


def _ensure_ready():
    global _noul
    if _noul is None:
        import von
        von.set_backend('modernbert')
        from von_batch import noul_batch
        _noul = noul_batch


def reset():
    """Clear inter-call state (error trend, gate). Call between runs."""
    global _e_prev, _t_last, _mu_last
    _e_prev = 0.0
    _t_last = None
    _mu_last = None


def tick(rpm, target, duty_prev, t):
    """One controller step at the model tick; inference gated by
    params.von.cadence_s — returns (duty, mu), holding both between
    inference calls."""
    _ensure_ready()
    global _e_prev, _t_last, _mu_last
    v = PARAMS['von']

    # t < _t_last means a new run restarted the clock -> re-arm the gate
    if _t_last is not None and 0 <= t - _t_last < v['cadence_s']:
        return duty_prev, (_mu_last or [0.0] * 9), False
    _t_last = t

    err = rpm - target
    trend = 'increasing' if err > _e_prev else 'decreasing'
    _e_prev = err
    state = (f'DC motor speed telemetry: target={target:.0f} RPM, '
             f'measured={rpm:.0f} RPM ({100 * rpm / target:.0f}% of target), '
             f'error={err:+.0f} RPM, error is {trend}.')

    mu = [float(g) for g in _noul(state, v['antecedents'])]
    den = sum(mu)
    delta = (sum(m * c for m, c in zip(mu, v['consequents'])) / den
             if den > 0 else 0.0)

    # error-magnitude throttle: near target the biased under-grades would
    # otherwise keep pushing past the plant lag
    delta *= min(1.0, abs(err) / v['err_fs'])

    # deadband: small integral leak inside +/-db_rpm
    if abs(err) <= v['db_rpm']:
        delta = -v['ki_leak'] * err

    # cadence invariance: consequents were tuned as duty-per-call at
    # 0.15 s; scale so the effective rate is cadence-independent
    delta *= v['cadence_s'] / 0.15

    _mu_last = mu
    return min(1.0, max(0.0, duty_prev + delta)), mu, True
