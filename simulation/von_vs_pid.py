"""Von-fuzzy vs discrete PI — pure-Python offline comparison.

Same loop as motor_von_hw.slx:
    plant and controller constants come from controller_params.json
    (via von_control.PARAMS) — single source of truth with the hardware
    model. Controller output is duty fraction [0,1]; volts = volts_per_duty
    * duty. Sensor noise enters only the feedback path.
    PI: 1-DOF, P on error, forward-Euler I, clamping anti-windup
    Von: von_control.tick at the JSON cadence (hardware gate)

Run: .venv/bin/python von_vs_pid.py
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'shared'))
import von_control

P = von_control.PARAMS
TS = P['plant']['ts']
A, B = P['plant']['a'], P['plant']['b']
T_END = 20.0
REF = P['ref_rpm']
VOLTS = P['volts_per_duty']
NOISE_SIGMA = P['noise_sigma_rpm']
SEED = 23341


def simulate(controller, rng):
    """Closed-loop step response. controller: 'pi' | 'von'."""
    n = round(T_END / TS)
    y = np.zeros(n)
    u_log = np.zeros(n)
    mu_log, mu_t = [], []
    x = 0.0                       # plant state = y[k]
    duty = 0.0
    integ = 0.0                   # PI integrator
    KP, I_TS = P['pi']['kp'], P['pi']['ki'] * TS
    alpha = np.exp(-TS / P['rpm_filter_tau'])   # RPMFilter, motor_von_hw
    yf = 0.0
    von_control.reset()
    for k in range(n):
        y[k] = x
        yf = alpha * yf + (1 - alpha) * (x + rng.normal(0.0, NOISE_SIGMA))
        rpm_meas = yf
        if controller == 'pi':
            e = REF - rpm_meas
            du = I_TS * e
            u_unsat = KP * e + integ + du
            duty = min(1.0, max(0.0, u_unsat))
            if not (u_unsat > 1 and du > 0) and not (u_unsat < 0 and du < 0):
                integ += du
        else:                     # von: gate handled inside tick()
            duty, mu, fired = von_control.tick(rpm_meas, REF, duty, k * TS)
            if fired:
                mu_log.append(mu); mu_t.append(k * TS)
        u_log[k] = duty
        x = A * x + B * (VOLTS * duty)
    t = np.arange(n) * TS
    return t, y, u_log, np.asarray(mu_log), np.asarray(mu_t)


def metrics(y, t, ref):
    i10 = np.argmax(y >= 0.1 * ref)
    i90 = np.argmax(y >= 0.9 * ref)
    rt = t[i90] - t[i10] if i90 > i10 else np.nan
    os_ = max(0.0, (y[t <= 6].max() - ref) / ref * 100)
    outside = np.abs(y - ref) > 0.02 * ref
    st = t[np.nonzero(outside)[0][-1]] + TS if outside.any() else t[0]
    ess = ref - y[-5:].mean()
    ys = y[t > max(10, t[-1] - 10)]
    return dict(rise=rt, overshoot=os_, settling=st,
                ss_err=ess, ripple=ys.max() - ys.min())


def main():
    rng = np.random.default_rng(SEED)
    t, y_pi, u_pi, _, _ = simulate('pi', rng)
    rng = np.random.default_rng(SEED)   # same noise stream for fairness
    t, y_von, u_von, mu_von, mu_t = simulate('von', rng)

    m_pi, m_von = metrics(y_pi, t, REF), metrics(y_von, t, REF)
    print(f"{'':24s} {'PI':>10s} {'Von':>10s}")
    for k in ('rise', 'overshoot', 'settling', 'ss_err', 'ripple'):
        print(f"{k:24s} {m_pi[k]:10.3f} {m_von[k]:10.3f}")

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    ax[0].plot(t, y_pi, 'b', label='Discrete PI')
    ax[0].plot(t, y_von, 'r', label='Von fuzzy')
    ax[0].axhline(REF, color='k', ls='--', label=f'{REF:g} RPM')
    ax[0].set_ylabel('Speed [RPM]'); ax[0].legend(); ax[0].grid()
    ax[0].set_title('Closed-loop step response (true speed)')
    ax[1].plot(t, u_pi * 255, 'b', label='PI PWM duty')
    ax[1].plot(t, u_von * 255, 'r', label='Von PWM duty')
    ax[1].set_xlabel('Time [s]'); ax[1].set_ylabel('PWM duty [0-255]')
    ax[1].legend(); ax[1].grid()
    fig.tight_layout(); fig.savefig('comparison.png', dpi=150)

    if len(mu_von):
        fig2, ax2 = plt.subplots(figsize=(9, 5))
        terms = ['far_under', 'under', 'slightly_under', 'near_under',
                 'on_target', 'near_over', 'slightly_over', 'over', 'far_over']
        ax2.step(mu_t, mu_von, where='post')
        ax2.legend(terms, loc='center left', bbox_to_anchor=(1, 0.5))
        ax2.set_xlabel('Time [s]'); ax2.set_ylabel('mu_i')
        ax2.set_title('Von Noul grades per tick'); ax2.grid()
        fig2.tight_layout(); fig2.savefig('von_memberships.png', dpi=150)

    np.savez('comparison_result.npz', t=t, y_pi=y_pi, y_von=y_von,
             u_pi=u_pi, u_von=u_von, mu_von=mu_von)
    print('wrote comparison.png, von_memberships.png, comparison_result.npz')


if __name__ == '__main__':
    main()
