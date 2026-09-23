"""Consequent retune for the pct-serialized antecedents.

Reuses simulate()/metrics() from von_vs_pid; overrides von params per
candidate. Prints a metric table for each.

Run: ../.venv/bin/python tune_consequents.py
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'shared'))
import von_control
import von_vs_pid as vp

BASE = list(von_control.PARAMS['von']['consequents'])
BASE_BF = von_control.PARAMS['von']['brake_floor']
BASE_KL = von_control.PARAMS['von']['ki_leak']

# candidates: (label, under, slightly_under, near_over, slightly_over,
#              over, brake_floor, mu_ema)
GRID = [
    ('best: u.10 su.03 | ov-.10 so-.20 o-.3 bf.4 ema0',
     0.10, 0.03, -0.10, -0.20, -0.30, 0.4, 0.0),
    ('same, ema.4',
     0.10, 0.03, -0.10, -0.20, -0.30, 0.4, 0.4),
    ('same, ema.6',
     0.10, 0.03, -0.10, -0.20, -0.30, 0.4, 0.6),
    ('same, ema.6 bf.3',
     0.10, 0.03, -0.10, -0.20, -0.30, 0.3, 0.6),
    ('ema.6 + stronger brakes so-.25 o-.35 bf.5',
     0.10, 0.03, -0.10, -0.25, -0.35, 0.5, 0.6),
]


def main():
    v = von_control.PARAMS['von']
    print(f"{'candidate':52s} {'rise':>6} {'os%':>6} {'setl':>6} "
          f"{'ss':>6} {'ripl':>6}")
    for label, u, su, nov, sov, ov, bf, ema in GRID:
        v['consequents'] = [0.25, u, su, 0.001, 0.0, nov, sov, ov, -0.25]
        v['brake_floor'] = bf
        v['mu_ema'] = ema
        rng = np.random.default_rng(vp.SEED)
        t, y, _, _, _ = vp.simulate('von', rng)
        m = vp.metrics(y, t, vp.REF)
        print(f"{label:52s} {m['rise']:6.2f} {m['overshoot']:6.2f} "
              f"{m['settling']:6.2f} {m['ss_err']:6.3f} {m['ripple']:6.3f}")
    v['consequents'] = BASE
    v['brake_floor'] = BASE_BF
    v['ki_leak'] = BASE_KL
    v.pop('mu_ema', None)


if __name__ == '__main__':
    main()
