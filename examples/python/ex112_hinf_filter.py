"""
ex112_hinf_filter.py

Phase 3 (EF1): HinfFilter vs. KalmanFilter under bounded impulsive disturbances.

Mirrors ex95_hinf_filter.cpp - a vibration-sensor-style scenario with occasional sharp
impulse disturbances, where KalmanFilter's Gaussian assumption has no worst-case guarantee
but HinfFilter does.
"""

import sys
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'HinfFilter'):
        raise AttributeError("HinfFilter not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

A = np.array([[0.9]])
B = np.array([[0.0]])
C = np.array([[1.0]])
D = np.array([[0.0]])
plant = ctrl.StateSpace(A, B, C, D, 0.1)

Qw = np.array([[0.01]])
Rv = np.array([[0.05]])

hf_result = ctrl.HinfFilter.solve(plant, Qw, Rv)
if not hf_result.feasible:
    print("[FAIL] HinfFilter synthesis infeasible")
    sys.exit(1)
print(f"HinfFilter achieved gamma = {hf_result.achieved_gamma:.4f}")

hf = ctrl.HinfFilter(hf_result)
kf = ctrl.KalmanFilter(plant, Qw, Rv)

rng = np.random.default_rng(42)
x_true = 0.0
sse_hf = 0.0
sse_kf = 0.0
N = 500
for k in range(N):
    w = rng.normal(0.0, np.sqrt(Qw[0, 0]))
    if k % 50 == 0:
        w += (0.5 if k % 100 == 0 else -0.5)
    v = rng.normal(0.0, np.sqrt(Rv[0, 0]))
    y = C[0, 0] * x_true + v

    hf.predict(np.array([0.0]))
    hf.update(np.array([y]))
    kf.step(np.array([y]), np.array([0.0]))

    sse_hf += (x_true - hf.state()[0]) ** 2
    sse_kf += (x_true - kf.state()[0]) ** 2

    x_true = A[0, 0] * x_true + w

rms_hf = np.sqrt(sse_hf / N)
rms_kf = np.sqrt(sse_kf / N)
print(f"RMS error under impulsive disturbance:  HinfFilter={rms_hf:.4f}  KalmanFilter={rms_kf:.4f}")

ok = np.isfinite(rms_hf) and np.isfinite(rms_kf) and rms_hf < 5.0 * rms_kf
print("[PASS] All checks passed." if ok else "[FAIL] One or more checks failed.")
sys.exit(0 if ok else 1)
