"""
ex120_set_membership_estimation.py

Phase 3 Roadmap Phase 2 (EF2): bounded-error ellipsoidal state estimation.

Mirrors ex103_set_membership_estimation.cpp - the set-membership ellipsoid never excludes the
true state under non-Gaussian (uniform-bounded) noise, unlike a KalmanFilter's confidence band.
"""

import sys
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'SetMembershipEstimator'):
        raise AttributeError("SetMembershipEstimator not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

plant = ctrl.StateSpace(np.array([[0.9]]), np.array([[1.0]]), np.array([[1.0]]),
                         np.array([[0.0]]), 0.1)

smp = ctrl.SetMembershipParams()
smp.w_bound, smp.v_bound = 0.05, 0.3
est = ctrl.SetMembershipEstimator(plant, smp, np.array([0.0]), np.eye(1))
kf = ctrl.KalmanFilter(plant, np.array([[smp.w_bound ** 2]]), np.array([[smp.v_bound ** 2 / 9.0]]))

rng = np.random.default_rng(9)
x_true = 0.0
u = np.array([0.2])
sm_contains, kf_contains = 0, 0
N = 200

for _ in range(N):
    x_true = 0.9 * x_true + u[0] + rng.uniform(-smp.w_bound, smp.w_bound)
    est.predict(u)
    kf.predict(u)
    y = np.array([x_true + rng.uniform(-smp.v_bound, smp.v_bound)])
    est.update(y)
    kf.update(y, u)

    d = np.array([x_true]) - est.center_estimate()
    quad = d @ np.linalg.solve(est.ellipsoid_shape(), d)
    if quad <= 1.0 + 1e-6:
        sm_contains += 1

    kf_sigma = np.sqrt(kf.covariance()[0, 0])
    if abs(x_true - kf.state()[0]) <= 3.0 * kf_sigma:
        kf_contains += 1

print(f"SetMembershipEstimator: ellipsoid contained the true state in {sm_contains}/{N} steps")
print(f"KalmanFilter: 3-sigma interval contained the true state in {kf_contains}/{N} steps")

ok = sm_contains == N
print("[PASS] All checks passed." if ok else "[FAIL] One or more checks failed.")
sys.exit(0 if ok else 1)
