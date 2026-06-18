"""
ex98_recursive_grey_box.py - E2: RecursiveGreyBoxEstimator - online augmented-UKF.

Demonstrates:
  1. Augmented-state UKF tracking one unknown plant parameter online.
  2. Parameter estimate converges from a wrong initial guess toward the true value.
  3. State estimate remains finite and close to the true state.

System: xdot = -p*x + u,  y = x.  True p = 0.5 (initial guess: 0.2).

Expected output:
  [PASS] Initialized correctly.
  [PASS] Param estimate finite and improved after 100 steps.
  [PASS] State estimate close to true state.
  [PASS] All checks passed.
"""

import sys, os
import _setup_bindings  # noqa: F401

import math
import numpy as np

try:
    import ctrl_toolbox as ctrl
    feats = ctrl.features()
    if not feats.get('recursive_grey_box', feats.get('advanced_kalman', True)):
        print("SKIP: recursive_grey_box feature not available (advanced_kalman disabled)")
        sys.exit(0)
    if not hasattr(ctrl, 'RecursiveGreyBoxEstimator'):
        raise AttributeError("RecursiveGreyBoxEstimator not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

TRUE_P = 0.5
Ts     = 0.05
rng    = np.random.default_rng(42)

def ode_fn(x, u, p):
    xd = np.empty(1)
    xd[0] = -p[0] * x[0] + u[0]
    return xd

def meas_fn(x, p):
    return x.copy()

par = ctrl.RecursiveGreyBoxParams()
par.p0      = np.array([0.2])        # wrong initial guess
par.Q_state = np.eye(1) * 1e-4
par.Q_param = np.eye(1) * 1e-5
par.R_meas  = np.eye(1) * 1e-3
par.Ts      = Ts
par.alpha   = 0.3
par.beta    = 2.0
par.kappa   = 0.0

rge = ctrl.RecursiveGreyBoxEstimator(ode_fn, meas_fn, 1, 1, par)

ok_init = not rge.is_initialized()
rge.initialize(np.zeros(1), np.eye(1) * 0.1)
ok_init = ok_init and rge.is_initialized()
print(f"[{'PASS' if ok_init else 'FAIL'}] Initialized correctly.")

# Simulate true system (Forward Euler)
xs = 0.0
p_history = []
for k in range(100):
    xs  += Ts * (-TRUE_P * xs + 1.0)
    y_k  = np.array([xs + 0.005 * rng.standard_normal()])
    u_k  = np.array([1.0])
    rge.step(y_k, u_k)
    p_history.append(float(rge.param_estimate()[0]))

p_final = float(rge.param_estimate()[0])
err_init = abs(0.2 - TRUE_P)
err_final = abs(p_final - TRUE_P)

print(f"  Initial p guess  : 0.2  (error {err_init:.3f})")
print(f"  Final p estimate : {p_final:.4f}  (error {err_final:.3f})")

ok_param = (math.isfinite(p_final) and err_final < err_init)
print(f"[{'PASS' if ok_param else 'FAIL'}] Param estimate finite and improved after 100 steps.")

x_hat = float(rge.state_estimate()[0])
ok_state = math.isfinite(x_hat) and abs(x_hat - xs) < 0.3
print(f"  True x = {xs:.4f}   x_hat = {x_hat:.4f}   |err| = {abs(x_hat-xs):.4f}")
print(f"[{'PASS' if ok_state else 'FAIL'}] State estimate close to true state.")

all_ok = ok_init and ok_param and ok_state
print(f"\n[{'PASS' if all_ok else 'FAIL'}] All checks passed.")
sys.exit(0 if all_ok else 1)
