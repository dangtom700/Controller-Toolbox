"""
ex86_koopman.py -- Koopman / Extended Dynamic Mode Decomposition (EDMD) demo.

Demonstrates:
  1. Collecting snapshots from a nonlinear (Duffing-like) plant.
  2. Fitting a Koopman linear model in the lifted polynomial space.
  3. Using the projected linear StateSpace with DiscreteLQR for tracking.

Plant:  x[k+1] = 0.9*x[k] - 0.05*x[k]^3 + 0.2*u[k]   (nonlinear, stable)
"""

import sys, os
import _setup_bindings  # noqa: F401

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'KoopmanEDMD'):
        raise AttributeError("KoopmanEDMD not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

import numpy as np

# ---------------------------------------------------------------------------
# Plant
# ---------------------------------------------------------------------------
def plant_step(x, u):
    return 0.9 * x - 0.05 * x**3 + 0.2 * u

# ---------------------------------------------------------------------------
# 1. Collect snapshots from random excitation
# ---------------------------------------------------------------------------
print("=== ex86: Koopman / EDMD ===")
print("\n-- Step 1: collecting 300 training snapshots --")

p = ctrl.KoopmanEDMDParams()
p.n_state = 1
p.n_input = 1
p.dict    = ctrl.KoopmanDict.PolyDeg2
p.tikhonov = 1e-8

edmd = ctrl.KoopmanEDMD(p)

x = 0.0
rng = np.random.default_rng(42)
for _ in range(300):
    u  = float(rng.uniform(-1.5, 1.5))
    x1 = plant_step(x, u)
    edmd.add_snapshot(np.array([x]), np.array([u]), np.array([x1]))
    x = x1

print(f"  Snapshots: {edmd.snapshot_count()},  lifted dim: {edmd.n_lifted()}")

# ---------------------------------------------------------------------------
# 2. Fit projected linear model
# ---------------------------------------------------------------------------
print("\n-- Step 2: fitting Koopman projected model --")
ss = edmd.fit_projected()
print(f"  A = {ss.A[0,0]:.4f}  (near 0.9 for small x)")
print(f"  B = {ss.B[0,0]:.4f}  (near 0.2)")

# ---------------------------------------------------------------------------
# 3. Open-loop prediction accuracy
# ---------------------------------------------------------------------------
print("\n-- Step 3: prediction vs. true plant (10 steps from x=0.5) --")
x_true = 0.5
x_lin  = 0.5
print(f"  k    x_true   x_linear")
for k in range(10):
    u = 0.3 * np.sin(k * 0.5)
    x_true = plant_step(x_true, u)
    x_lin  = ss.A[0, 0] * x_lin + ss.B[0, 0] * u
    print(f"  {k+1:2d}   {x_true:8.4f}  {x_lin:8.4f}")

# ---------------------------------------------------------------------------
# 4. LQR on linearised model
# ---------------------------------------------------------------------------
print("\n-- Step 4: DiscreteLQR on Koopman model (step tracking) --")
import numpy as np

lqr_p = ctrl.LQRParams()
lqr_p.Q = np.array([[10.0]])
lqr_p.R = np.array([[0.1]])
lqr = ctrl.DiscreteLQR(ss, lqr_p)
K = lqr.gain_matrix()
print(f"  LQR gain K = {K[0,0]:.4f}")

x = 0.0
r = 1.0
iae = 0.0
for k in range(200):
    e = r - x
    u = float(K[0, 0] * e)
    x = plant_step(x, u)
    iae += abs(r - x) * 0.01

print(f"  IAE (200 steps, Ts=0.01): {iae:.4f}")
print(f"  Final x: {x:.4f}  (target: {r:.1f})")
print("\n[PASS] KoopmanEDMD demo complete.")
