"""
ex144_resonant_controller.py

ResonantController - single-harmonic internal-model corrector, composed through a
ControllerStack(Additive) alongside a base PID (one resonant term per harmonic).

A 2.5 Hz sinusoidal disturbance enters at the plant input. PID alone leaves a
steady-state ripple at that frequency; adding a resonant corrector tuned to
2.5 Hz places infinite loop gain there and cancels the ripple. The stack (not a
manual sum) does the composition.

Mirrors ex89_resonant_controller.cpp (single harmonic, shorter run).
"""

import sys
import math
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'ResonantController'):
        raise AttributeError("ResonantController not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

Ts = 1e-3
plant = ctrl.c2d(ctrl.StateSpace(np.array([[-1.0]]), np.array([[1.0]]),
                                 np.array([[1.0]]), np.zeros((1, 1)), 0.0),
                 Ts, ctrl.C2dMethod.ZOH)   # G(s) = 1/(s+1)

F_DIST = 2.5          # Hz
N = 20000             # 20 s = 50 cycles of the disturbance
CYCLE = int(round(1.0 / F_DIST / Ts))   # steps per disturbance cycle (400)
ref = 1.0


def pid_params():
    p = ctrl.PIDParams()
    p.Kp, p.Ki = 2.0, 5.0
    p.uMin, p.uMax = -1e6, 1e6
    return p


def disturbance(k):
    return 0.3 * math.sin(2.0 * math.pi * F_DIST * k * Ts)


def run(use_resonant):
    pid = ctrl.DiscretePID(pid_params(), Ts)
    stack = ctrl.ControllerStack(ctrl.StackMode.Additive, Ts)
    stack.add_controller(pid, "PID-base", weight=1.0, condition=None)
    if use_resonant:
        rp = ctrl.ResonantParams()
        rp.targetFreqHz = F_DIST
        rp.dampingRadPerSec = 5.0
        rp.Kr = 200.0
        rc = ctrl.ResonantController(rp, Ts)
        stack.add_controller(rc, "2.5Hz-resonant", weight=1.0, condition=None)

    x = np.zeros(plant.state_size())
    y = 0.0
    ripple = 0.0
    for k in range(N):
        u = stack.compute(ref - y)
        y_vec, x = ctrl.ss_step_copy(plant, x, np.array([u + disturbance(k)]))
        y = float(np.squeeze(y_vec))
        if k >= N - CYCLE:            # max |error| over the final disturbance cycle
            ripple = max(ripple, abs(ref - y))
    return ripple


ripple_pid = run(use_resonant=False)
ripple_res = run(use_resonant=True)
print("=== ResonantController vs PID-only (2.5 Hz input disturbance) ===")
print(f"  PID-only       last-cycle ripple |e|max = {ripple_pid:.4f}")
print(f"  PID + resonant last-cycle ripple |e|max = {ripple_res:.4f}")
print(f"  reduction = {(1.0 - ripple_res / ripple_pid) * 100:.1f}%")

ok = (np.isfinite(ripple_res) and np.isfinite(ripple_pid)
      and ripple_res < 0.4 * ripple_pid)
print("[PASS]" if ok else "[FAIL]")
sys.exit(0 if ok else 1)
