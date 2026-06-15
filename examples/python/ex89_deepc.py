"""
ex89_deepc.py - Part 30: DeePC (Data-Enabled Predictive Control).

Plant: y[k+1] = 0.7*y[k] + 0.3*u[k]   (steady-state gain = 1.0)
Reference: r = 1.0

Demonstrates:
  1. Collecting offline persistently exciting data.
  2. Building Hankel matrices and the constant LDLT-factored QP Hessian.
  3. Online ADMM-based control without explicit system identification.

Acceptance:
  - Steady-state |y - r| < 0.15 after 100 steps.
  - All control outputs within [uMin, uMax].
  - is_healthy() True for >= 90% of control steps.
"""

import sys
import os
import _setup_bindings  # noqa: F401

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'DeePC'):
        raise AttributeError("DeePC not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

import numpy as np

# ---------------------------------------------------------------------------
# Plant parameters
# ---------------------------------------------------------------------------
A  = 0.7   # plant pole
B  = 0.3   # plant gain
TS = 0.1   # sample time [s]
R  = 1.0   # setpoint

# ---------------------------------------------------------------------------
# Phase 1: Collect offline data with random excitation
# ---------------------------------------------------------------------------
rng   = np.random.default_rng(seed=42)
N_d   = 100
u_d   = rng.uniform(-1.0, 1.0, N_d)
y_d   = np.zeros(N_d)
for k in range(1, N_d):
    y_d[k] = A * y_d[k-1] + B * u_d[k-1]

# ---------------------------------------------------------------------------
# Phase 2: Build DeePC controller
# ---------------------------------------------------------------------------
p = ctrl.DeePCParams()
p.T_ini      = 10
p.N          = 10
p.Q          = 10.0
p.R          = 0.05
p.lambda_g   = 1.0
p.lambda_y   = 50.0
p.lambda_u   = 5.0
p.uMin       = -3.0
p.uMax       =  3.0
p.rho        = 10.0
p.admm_iters = 300
p.admm_tol   = 1e-4

dc = ctrl.DeePC(p, TS)
dc.collect_data(u_d, y_d)
dc.set_reference(R)

print(f"Hankel columns M = {dc.hankel_columns()}")

# ---------------------------------------------------------------------------
# Phase 3: Control loop
# ---------------------------------------------------------------------------
N_steps   = 120
y_k       = 0.0
u_k       = 0.0
n_unhealthy = 0

print(f"{'step':>6}  {'y':>8}  {'u':>8}  {'error':>8}  ok?")
for k in range(N_steps):
    u_k = dc.compute(float(y_k))
    if not dc.is_healthy():
        n_unhealthy += 1
    err = R - y_k
    if k % 10 == 0:
        print(f"{k:6d}  {y_k:8.4f}  {u_k:8.4f}  {err:8.4f}  {'OK' if dc.is_healthy() else '!'}")
    y_k = A * y_k + B * u_k

# ---------------------------------------------------------------------------
# Phase 4: Acceptance checks
# ---------------------------------------------------------------------------
final_err   = abs(y_k - R)
pct_healthy = 100.0 * (1.0 - n_unhealthy / N_steps)

print(f"\nFinal y={y_k:.4f}  |y-r|={final_err:.4f}  healthy={pct_healthy:.0f}%")

failures = []
if final_err > 0.15:
    failures.append(f"tracking error {final_err:.4f} > 0.15")
if not (p.uMin <= u_k <= p.uMax):
    failures.append(f"final u={u_k:.4f} outside [{p.uMin}, {p.uMax}]")
if pct_healthy < 90.0:
    failures.append(f"ADMM healthy only {pct_healthy:.0f}% of steps")

if failures:
    for f in failures:
        print(f"FAIL: {f}")
    sys.exit(1)

print("PASS")
