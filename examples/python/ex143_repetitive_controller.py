"""
ex143_repetitive_controller.py

RepetitiveController - internal-model rejection of a PERIODIC disturbance.

It wraps a stabilising PID and adds a learned correction that, over successive
periods, converges to the negative of the repeating disturbance. Compared here
against PID-only: the RC collapses the steady-state ripple at the disturbance
period.

Mirrors ex29_repetitive_controller.cpp (with a clean input-disturbance injection).
"""

import sys
import math
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'RepetitiveController'):
        raise AttributeError("RepetitiveController not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

Ts = 0.1
tau, Kp_plant = 5.0, 2.0
a = math.exp(-Ts / tau)
b = Kp_plant * (1.0 - a)             # first-order plant y[k+1] = a*y + b*(u + d)

P = 100                              # disturbance period in steps (10 s)
ref = 1.0
N = P * 8                            # 8 periods


def pid_params():
    p = ctrl.PIDParams()
    p.Kp, p.Ki, p.N = 1.5, 0.3, 10.0
    p.uMin, p.uMax = -10.0, 10.0
    return p


def disturbance(k):
    return 0.5 * math.sin(2.0 * math.pi * k / P)


def run(use_rc):
    pid = ctrl.DiscretePID(pid_params(), Ts)
    if use_rc:
        rp = ctrl.RepetitiveParams()
        rp.period_steps = P
        rp.Krc, rp.Q = 0.4, 0.98
        rp.uMin, rp.uMax = -10.0, 10.0
        ctl = ctrl.RepetitiveController(pid, rp, Ts)
    else:
        ctl = pid

    y = 0.0
    ripple = 0.0
    for k in range(N):
        u = ctl.compute(ref - y)
        y = a * y + b * (u + disturbance(k))
        if k >= N - P:                # max |error| over the final period
            ripple = max(ripple, abs(ref - y))
    return ripple


ripple_pid = run(use_rc=False)
ripple_rc = run(use_rc=True)
print("=== RepetitiveController vs PID-only (periodic disturbance) ===")
print(f"  PID-only  last-period ripple |e|max = {ripple_pid:.4f}")
print(f"  PID + RC  last-period ripple |e|max = {ripple_rc:.4f}")
print(f"  reduction = {(1.0 - ripple_rc / ripple_pid) * 100:.1f}%")

ok = (np.isfinite(ripple_rc) and np.isfinite(ripple_pid)
      and ripple_rc < 0.5 * ripple_pid)
print("[PASS]" if ok else "[FAIL]")
sys.exit(0 if ok else 1)
