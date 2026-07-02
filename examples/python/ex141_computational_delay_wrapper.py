"""
ex141_computational_delay_wrapper.py

ComputationalDelayWrapper - one-sample computational-delay decorator.

The SAME PID on the SAME lightly-damped plant, with vs without a one-sample
computational delay. The delay erodes phase margin -> larger overshoot, while
both loops still settle (integral action). The wrapper EXPOSES the deployed
loop's margin cost; it does not fix it.

Mirrors ex125_computational_delay_wrapper.cpp.
"""

import sys
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'ComputationalDelayWrapper'):
        raise AttributeError("ComputationalDelayWrapper not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

Ts = 0.1
# Lightly-damped 2nd-order plant (wn=1, zeta=0.3), ZOH-discretised.
Ac = np.array([[0.0, 1.0], [-1.0, -0.6]])
Bc = np.array([[0.0], [1.0]])
Cc = np.array([[1.0, 0.0]])
Dc = np.zeros((1, 1))
plant = ctrl.c2d(ctrl.StateSpace(Ac, Bc, Cc, Dc, 0.0), Ts, ctrl.C2dMethod.ZOH)


def run(with_delay):
    # Gentle gains so BOTH loops settle; the delay's cost then shows as extra overshoot.
    p = ctrl.PIDParams()
    p.Kp, p.Ki, p.Kd, p.N = 0.7, 0.7, 0.12, 20.0
    p.uMin, p.uMax = -1e6, 1e6
    pid = ctrl.DiscretePID(p, Ts)
    ctl = ctrl.ComputationalDelayWrapper(pid, 0.0) if with_delay else pid

    N, WIN = 1000, 100
    x = np.zeros(plant.state_size())
    y = 0.0
    peak, tail = 0.0, 0.0
    for k in range(N):
        u = ctl.compute(1.0 - y)      # PID sign convention: r - y
        y_vec, x = ctrl.ss_step_copy(plant, x, np.array([u]))
        y = float(np.squeeze(y_vec))
        peak = max(peak, y)
        if k >= N - WIN:
            tail += y
    return peak, tail / WIN


peak_nd, set_nd = run(with_delay=False)
peak_d,  set_d  = run(with_delay=True)
print(f"no delay : peak={peak_nd:.3f}  overshoot={(peak_nd-1)*100:.1f}%  settled={set_nd:.3f}")
print(f"1-sample : peak={peak_d:.3f}  overshoot={(peak_d-1)*100:.1f}%  settled={set_d:.3f}")

ok = (np.isfinite(set_d) and np.isfinite(set_nd)
      and peak_d > peak_nd
      and abs(set_d - 1.0) < 0.05 and abs(set_nd - 1.0) < 0.05)
print("[PASS]" if ok else "[FAIL]")
sys.exit(0 if ok else 1)
