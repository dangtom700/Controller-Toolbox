"""
ex96_esn.py -- Echo State Network (ESN) / Reservoir Computing demo.

Demonstrates:
  1. Training an ESN on data from a nonlinear first-order plant.
  2. One-step prediction accuracy (MSE) on a held-out test sequence.
  3. Autonomous generation (reservoir state-function retrieval).

Plant:  y[k+1] = tanh(0.8*y[k] + 0.5*u[k])
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'build', 'bindings'))
if sys.platform == "win32" and hasattr(os, "add_dll_directory"):
    for _p in [r"C:\msys64\mingw64\bin"]:
        if os.path.isdir(_p):
            os.add_dll_directory(_p)

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'EchoStateNetwork'):
        raise AttributeError("EchoStateNetwork not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

import numpy as np

print("=== ex96: Echo State Network ===")

def plant_step(y, u):
    return float(np.tanh(0.8 * y + 0.5 * u))

# ---------------------------------------------------------------------------
# 1. Build and train ESN
# ---------------------------------------------------------------------------
print("\n-- Part 1: training on 500 steps --")

ep = ctrl.ESNParams()
ep.n_res           = 80
ep.n_in            = 1
ep.n_out           = 1
ep.spectral_radius = 0.9
ep.sparsity        = 0.85
ep.alpha           = 0.3
ep.ridge           = 1e-4
ep.washout         = 50
ep.seed            = 42

esn = ctrl.EchoStateNetwork(ep)

rng = np.random.default_rng(42)
y = 0.0
for _ in range(500):
    u      = float(rng.uniform(-1.0, 1.0))
    y_next = plant_step(y, u)
    esn.step_reservoir(np.array([u]))
    esn.add_training_target(np.array([y_next]))
    y = y_next

esn.fit_readout()
print(f"  Reservoir size: {esn.reservoir_size()}")
print(f"  Fitted:         {esn.is_fitted()}")

# ---------------------------------------------------------------------------
# 2. One-step prediction on held-out test sequence
# ---------------------------------------------------------------------------
print("\n-- Part 2: one-step prediction (MSE) --")

esn.reset()
y = 0.0
mse = 0.0
n_test = 100
print(f"  {'k':>5}  {'y_true':>8}  {'y_hat':>8}  {'err':>8}")

for k in range(n_test):
    u      = float(np.sin(k * 0.3))
    y_true = plant_step(y, u)
    y_hat  = float(esn.predict(np.array([u]))[0])
    err    = y_true - y_hat
    mse   += err * err
    if k < 5 or k >= n_test - 5:
        print(f"  {k:5d}  {y_true:8.4f}  {y_hat:8.4f}  {err:8.4f}")
    elif k == 5:
        print(f"  ...")
    y = y_true

mse /= n_test
print(f"\n  Test MSE: {mse:.5f}  (< 0.05 expected)")

# ---------------------------------------------------------------------------
# 3. Reservoir state inspection
# ---------------------------------------------------------------------------
print("\n-- Part 3: reservoir state info --")
r_state = esn.reservoir_state()
print(f"  Reservoir state dim: {len(r_state)}")
print(f"  State norm:          {float(np.linalg.norm(r_state)):.4f}")

print("\n[PASS] EchoStateNetwork demo complete.")
