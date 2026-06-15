"""
ex97_grey_box_estimator.py - E1: GreyBoxEstimator - batch LM parameter estimation.

Demonstrates:
  1. Defining ODE f(x, u, p) and measurement h(x, p) as Python callables.
  2. Generating a ground-truth trajectory for a first-order system.
  3. GreyBoxEstimator.fit() recovering true parameters from noisy output.
  4. GreyBoxEstimator.predict() checking trajectory reproduction.

Expected output:
  [PASS] Fitted parameters close to truth.
  [PASS] Predict output shape correct.
  [PASS] All checks passed.
"""

import sys, os
import _setup_bindings  # noqa: F401

import math
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'GreyBoxEstimator'):
        raise AttributeError("GreyBoxEstimator not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

# ---------------------------------------------------------------------------
# Ground-truth system: xdot = -a*x + b*u,  y = x
# ---------------------------------------------------------------------------
TRUE_A = 0.5
TRUE_B = 1.0
Ts     = 0.05
N      = 80

# Simulate with Forward Euler
xs = 0.0
U  = np.ones((1, N))
Y  = np.zeros((1, N))
for k in range(N):
    Y[0, k] = xs + 0.002 * ((k % 7) - 3)  # small noise
    xs += Ts * (-TRUE_A * xs + TRUE_B * U[0, k])

# ---------------------------------------------------------------------------
# Define ODE and measurement functions
# ---------------------------------------------------------------------------
def ode_fn(x, u, p):
    """First-order: xdot = -p[0]*x + p[1]*u"""
    xd = np.empty(1)
    xd[0] = -p[0] * x[0] + p[1] * u[0]
    return xd

def meas_fn(x, p):
    """Identity measurement"""
    return x.copy()

# ---------------------------------------------------------------------------
# Fit
# ---------------------------------------------------------------------------
par = ctrl.GreyBoxParams()
par.p0    = np.array([0.2, 0.6])
par.lower = np.array([0.01, 0.1])
par.upper = np.array([5.0,  5.0])
par.Ts        = Ts
par.max_iter  = 100
par.lm_lambda0 = 1e-3

est = ctrl.GreyBoxEstimator(ode_fn, meas_fn, par)
x0  = np.zeros(1)
res = est.fit(x0, U, Y)

p_hat = est.params()
print(f"  converged:  {res.converged}")
print(f"  iterations: {res.iterations}   cost: {res.cost:.6g}")
print(f"  a_hat = {p_hat[0]:.4f}  (true {TRUE_A})")
print(f"  b_hat = {p_hat[1]:.4f}  (true {TRUE_B})")

ok_a  = abs(p_hat[0] - TRUE_A) < 0.15
ok_b  = abs(p_hat[1] - TRUE_B) < 0.15

print(f"[{'PASS' if ok_a and ok_b else 'FAIL'}] Fitted parameters close to truth.")

# ---------------------------------------------------------------------------
# Predict shape check
# ---------------------------------------------------------------------------
Y_hat = est.predict(x0, U)
ok_shape = Y_hat.shape == (1, N) and np.all(np.isfinite(Y_hat))
print(f"[{'PASS' if ok_shape else 'FAIL'}] Predict output shape correct.")

all_ok = ok_a and ok_b and ok_shape
print(f"\n[{'PASS' if all_ok else 'FAIL'}] All checks passed.")
sys.exit(0 if all_ok else 1)
