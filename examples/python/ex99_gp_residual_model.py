"""
ex99_gp_residual_model.py - E3: GPResidualModel - GP mismatch correction for risk-aware MPC.

Demonstrates:
  1. Adding residual (y_true - y_model) data points online.
  2. Batch residualFit() from a matrix of feature vectors.
  3. predictWithUncertainty(): total, gp_mean, variance.
  4. Variance is lower near training data (GP interpolation property).

System: nominal model under-predicts by a sinusoidal mismatch ~0.1*sin(2*pi*x).

Expected output:
  [PASS] isFitted after fit().
  [PASS] mean_total = model_pred + gp_mean.
  [PASS] Lower variance near training data than far from it.
  [PASS] Batch residualFit matches manual add+fit.
  [PASS] All checks passed.
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'build', 'bindings'))
if sys.platform == "win32" and hasattr(os, "add_dll_directory"):
    for _p in [r"C:\msys64\mingw64\bin"]:
        if os.path.isdir(_p):
            os.add_dll_directory(_p)

import math
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'GPResidualModel'):
        raise AttributeError("GPResidualModel not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

# ---------------------------------------------------------------------------
# Build GPResidualModel
# ---------------------------------------------------------------------------
par = ctrl.GPResidualParams()
par.gp.length_scale = 0.4
par.gp.signal_var   = 0.05
par.gp.noise_var    = 1e-3
par.gp.n_max        = 100

grm = ctrl.GPResidualModel(1, par)

N_train = 20
X_train = np.linspace(0.0, 2.0, N_train).reshape(1, N_train)  # (1 x N)
# True residual: ~0.1*sin(2*pi*x)
Y_true   = 1.0 + 0.1 * np.sin(2 * math.pi * X_train[0])
Y_model  = np.ones(N_train)  # nominal model predicts 1.0

# Add points manually
for k in range(N_train):
    xf = X_train[:, k]         # shape (1,)
    grm.add_residual_point(xf, float(Y_true[k]), float(Y_model[k]))

grm.fit()

ok_fitted = grm.is_fitted()
print(f"[{'PASS' if ok_fitted else 'FAIL'}] isFitted after fit().")

# ---------------------------------------------------------------------------
# Check mean_total = model_pred + gp_mean
# ---------------------------------------------------------------------------
xf_test  = np.array([1.0])
model_v  = 1.0
pred     = grm.predict_with_uncertainty(xf_test, model_v)
ok_total = abs(pred.mean_total - (model_v + pred.gp_mean)) < 1e-10
print(f"  mean_total={pred.mean_total:.6f}  model={model_v}  gp_mean={pred.gp_mean:.6f}")
print(f"[{'PASS' if ok_total else 'FAIL'}] mean_total = model_pred + gp_mean.")

# ---------------------------------------------------------------------------
# Variance near training data < variance far away
# ---------------------------------------------------------------------------
pred_near = grm.predict_with_uncertainty(np.array([1.0]),  model_v)
pred_far  = grm.predict_with_uncertainty(np.array([10.0]), model_v)
ok_var    = pred_far.variance > pred_near.variance
print(f"  var near x=1.0: {pred_near.variance:.6f}   var far x=10.0: {pred_far.variance:.6f}")
print(f"[{'PASS' if ok_var else 'FAIL'}] Lower variance near training data than far from it.")

# ---------------------------------------------------------------------------
# Batch residualFit vs. manual add+fit
# ---------------------------------------------------------------------------
def model_fn(x_feat):
    return 1.0  # constant nominal model

grm2 = ctrl.GPResidualModel(1, par)
grm2.residual_fit(X_train, Y_true, model_fn)

pred1 = grm.predict_with_uncertainty(np.array([0.5]),  1.0)
pred2 = grm2.predict_with_uncertainty(np.array([0.5]), 1.0)
ok_batch = abs(pred1.mean_total - pred2.mean_total) < 1e-8
print(f"  manual mean_total={pred1.mean_total:.8f}  batch mean_total={pred2.mean_total:.8f}")
print(f"[{'PASS' if ok_batch else 'FAIL'}] Batch residualFit matches manual add+fit.")

all_ok = ok_fitted and ok_total and ok_var and ok_batch
print(f"\n[{'PASS' if all_ok else 'FAIL'}] All checks passed.")
sys.exit(0 if all_ok else 1)
