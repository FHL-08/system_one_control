"""Membership-function sweep: audits how coherent Von's grade structure
is for a given premise/antecedent wording.

Holds the antecedent questions fixed per variant; varies how the
measured speed is serialized in the premise:

  'ratio'  measured=32 RPM (80% of target)
  'pct'    measured=32 RPM (20% below the target)

and two phrasings of the range antecedents ("below by 10% to 30%" vs
"between 10% and 30% below"). For each rpm in the sweep each variant's
premise is evaluated with its antecedents in one batch. Reports, per
term, the separation between mean in-band and out-of-band grades, plus
argmax-band accuracy. Writes membership_sweep.png.

Run: .venv/bin/python simulation/membership_sweep.py
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'shared'))
import von_control

P = von_control.PARAMS
TARGET = P['ref_rpm']
# ratio-domain antecedents, paired with the 'N% of target' premise
ANTECEDENTS = [
    'Is the motor speed less than 70% of the target?',
    'Is the motor speed below the target by 10% to 30%?',
    'Is the motor speed below the target by 3% to 10%?',
    'Is the motor speed below the target by less than 3%?',
    'Is the motor speed at the target?',
    'Is the motor speed above the target by less than 3%?',
    'Is the motor speed above the target by 3% to 10%?',
    'Is the motor speed above the target by 10% to 40%?',
    'Is the motor speed more than 140% of the target?',
]
TERMS = ['far_under', 'under', 'slightly_under', 'near_under', 'on_target',
         'near_over', 'slightly_over', 'over', 'far_over']

# intended bands as signed % deviation from target:
#   far_under < -30, under [-30,-10], slightly_under [-10,-3],
#   near_under [-3,0), on_target ~0, near_over (0,3],
#   slightly_over (3,10], over (10,40], far_over > 40
EDGES = [-np.inf, -30, -10, -3, 0, 0, 3, 10, 40, np.inf]


def band_idx(pct):
    """Intended term index for a signed % deviation."""
    if abs(pct) < 1e-9:
        return 4                      # on_target
    for i in range(9):
        lo, hi = EDGES[i], EDGES[i + 1]
        if i < 4 and lo <= pct < hi:
            return i
        if i > 4 and lo < pct <= hi:
            return i
    return 4


def state_ratio(rpm, target, trend='steady'):
    """Ratio-domain premise."""
    err = rpm - target
    return (f'DC motor speed telemetry: target={target:.0f} RPM, '
            f'measured={rpm:.0f} RPM ({100 * rpm / target:.0f}% of target), '
            f'error={err:+.0f} RPM, error is {trend}.')


def state_pct(rpm, target, trend='steady'):
    """Deviation-domain premise (matches von_control.tick)."""
    err = rpm - target
    pct = 100 * err / target
    rel = ('at the target' if abs(pct) < 0.5 else
           f'{abs(pct):.0f}% below the target' if pct < 0 else
           f'{pct:.0f}% above the target')
    return (f'DC motor speed telemetry: target={target:.0f} RPM, '
            f'measured={rpm:.0f} RPM ({rel}), '
            f'error={err:+.0f} RPM, error is {trend}.')


# deviation-domain antecedents — the open-ended bands use single-sided
# threshold wording ("more than 30% below"), same units as the pct premise
ANTECEDENTS_PCT = [
    'Is the motor speed more than 30% below the target?',
    'Is the motor speed below the target by 10% to 30%?',
    'Is the motor speed below the target by 3% to 10%?',
    'Is the motor speed below the target by less than 3%?',
    'Is the motor speed at the target?',
    'Is the motor speed above the target by less than 3%?',
    'Is the motor speed above the target by 3% to 10%?',
    'Is the motor speed above the target by 10% to 40%?',
    'Is the motor speed more than 40% above the target?',
]

# same deviation domain, but range questions phrased as "between X and Y"
# to force the upper bound
ANTECEDENTS_BETWEEN = [
    'Is the motor speed more than 30% below the target?',
    'Is the motor speed between 10% and 30% below the target?',
    'Is the motor speed between 3% and 10% below the target?',
    'Is the motor speed below the target by less than 3%?',
    'Is the motor speed at the target?',
    'Is the motor speed above the target by less than 3%?',
    'Is the motor speed between 3% and 10% above the target?',
    'Is the motor speed between 10% and 40% above the target?',
    'Is the motor speed more than 40% above the target?',
]


def sweep(state_fn, rpms, antecedents=ANTECEDENTS):
    von_control._ensure_ready()
    noul = von_control._noul
    out = np.zeros((len(rpms), len(antecedents)))
    for i, r in enumerate(rpms):
        out[i] = noul(state_fn(r, TARGET), antecedents)
    return out


def report(name, mu, pcts):
    bands = np.array([band_idx(p) for p in pcts])
    # argmax accuracy: does the highest-grade term match the band?
    acc = (mu.argmax(axis=1) == bands).mean()
    print(f"\n{name}: argmax-in-band accuracy = {acc:.2%}")
    print(f"{'term':>15} {'in-band':>8} {'out-band':>8} {'sep':>6}")
    for i, term in enumerate(TERMS):
        inb = bands == i
        mi = mu[inb, i].mean() if inb.any() else np.nan
        mo = mu[~inb, i].mean()
        print(f"{term:>15} {mi:8.3f} {mo:8.3f} {mi - mo:6.3f}")


def main():
    rpms = np.arange(0, 61, 0.5)
    pcts = 100 * (rpms - TARGET) / TARGET

    print('sweeping ratio premise + ratio antecedents...')
    mu_ratio = sweep(state_ratio, rpms)
    print('sweeping pct premise + ratio antecedents...')
    mu_pct = sweep(state_pct, rpms)
    print('sweeping pct premise + threshold antecedents...')
    mu_pctq = sweep(state_pct, rpms, ANTECEDENTS_PCT)
    print('sweeping pct premise + between antecedents...')
    mu_bet = sweep(state_pct, rpms, ANTECEDENTS_BETWEEN)

    report('ratio premise + ratio antecedents', mu_ratio, pcts)
    report('pct premise + ratio antecedents', mu_pct, pcts)
    report('pct premise + threshold antecedents', mu_pctq, pcts)
    report('pct premise + between antecedents', mu_bet, pcts)

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(4, 1, figsize=(10, 14), sharex=True)
    band_edges = [-30, -10, -3, 0, 3, 10, 40]
    for ax, mu, title in zip(
            axes, [mu_ratio, mu_pct, mu_pctq, mu_bet],
            ['ratio premise "80% of target" + ratio antecedents',
             'pct premise "20% below target" + ratio antecedents',
             'pct premise + threshold antecedents ("more than 30% below")',
             'pct premise + between antecedents ("between 10% and 30%")']):
        for i, term in enumerate(TERMS):
            ax.plot(pcts, mu[:, i], label=term, lw=1.2)
        for e in band_edges:
            ax.axvline(e, color='k', ls=':', lw=0.6, alpha=0.5)
        ax.axvline(0, color='k', ls='-', lw=0.8)
        ax.set_ylabel('mu_i'); ax.set_ylim(-0.02, 1.02)
        ax.set_title(title); ax.grid(alpha=0.3)
    axes[0].legend(loc='center left', bbox_to_anchor=(1, 0.5), fontsize=8)
    axes[3].set_xlabel('signed deviation from target [%]')
    fig.tight_layout()
    fig.savefig('membership_sweep.png', dpi=150)
    np.savez('membership_sweep.npz', rpms=rpms, pcts=pcts,
             mu_ratio=mu_ratio, mu_pct=mu_pct, mu_pctq=mu_pctq,
             mu_bet=mu_bet)
    print('\nwrote membership_sweep.png, membership_sweep.npz')


if __name__ == '__main__':
    main()
