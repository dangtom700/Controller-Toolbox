"""
ex109_correlation_id.py

Phase 3 (SI2): cross-correlation impulse-response identification.

Drives a known first-order plant with a PRBS test signal, estimates its impulse response
via CorrelationID.identify(), and compares against the plant's true impulse response
(obtained directly by stepping a unit impulse through the same StateSpace).
"""

import sys
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'CorrelationID'):
        raise AttributeError("CorrelationID not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

Ts = 0.01
tf_true = ctrl.TransferFunction([0.0, 0.2], [1.0, -0.8], Ts)
plant = ctrl.tf2ss(tf_true)

N = 4000
u = ctrl.CorrelationID.generate_prbs(N, 10, seed=7)

y = np.zeros(N)
x = np.zeros(plant.A.shape[0])
for k in range(N):
    y_vec, x = ctrl.ss_step_copy(plant, x, np.array([u[k]]))
    y[k] = y_vec[0]

params = ctrl.CorrelationIDParams()
params.max_lag = 15
result = ctrl.CorrelationID.identify(u, y, Ts, params)

# True impulse response: step a unit impulse through a fresh instance of the same plant.
g_true = np.zeros(params.max_lag + 1)
x_imp = np.zeros(plant.A.shape[0])
for k in range(params.max_lag + 1):
    u_imp = np.array([1.0 if k == 0 else 0.0])
    g_vec, x_imp = ctrl.ss_step_copy(plant, x_imp, u_imp)
    g_true[k] = g_vec[0]

max_err = float(np.max(np.abs(result.impulse_response - g_true)))
print("lag   g_hat        g_true")
for k in range(params.max_lag + 1):
    print(f"  {k}   {result.impulse_response[k]:.5f}   {g_true[k]:.5f}")
print(f"Max abs error vs. true impulse response: {max_err:.4e}")

ok = np.isfinite(max_err) and max_err < 0.05
print("[PASS] All checks passed." if ok else "[FAIL] One or more checks failed.")
sys.exit(0 if ok else 1)
