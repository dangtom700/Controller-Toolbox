"""
ex126_nn_adaptive_control.py

Phase 3 (ML2): NN-adaptive control with online output-weight adaptation.

Mirrors ex109_nn_adaptive_control.cpp - a fixed tanh hidden layer plus an adapting linear
output layer regulates a plant with an unknown static input nonlinearity to a first-order
reference model.
"""

import sys
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'NNAdaptiveController'):
        raise AttributeError("NNAdaptiveController not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

Ts = 0.01

hidden = ctrl.NNLayerSpec()
hidden.W = np.array([[1.0, 0.5], [-0.8, 0.3], [0.6, -0.4],
                      [-0.5, 0.7], [0.9, -0.2], [0.2, 0.8]])
hidden.b = np.zeros(6)
hidden.activation = ctrl.NNLayerSpec.Activation.Tanh

out = ctrl.NNLayerSpec()
out.W = np.zeros((1, 6))
out.b = np.zeros(1)
out.activation = ctrl.NNLayerSpec.Activation.Linear

nn_params = ctrl.NeuralControllerParams()
nn_params.layers = [hidden, out]
nn_params.n_input_features = 2

params = ctrl.NNAdaptiveParams()
params.nn = nn_params
params.gamma_adapt = 3.0
params.sigma_mod = 0.01
params.a_m = 0.6
params.b_m = 0.4
params.uMin = -50.0
params.uMax = 50.0
c = ctrl.NNAdaptiveController(params, Ts)

r = 1.0
y = 0.0
y_m = 0.0
max_err_late = 0.0
for k in range(12000):
    c.set_reference(r)
    u = c.compute(y)
    y = 0.9 * y + 0.1 * (u + 0.3 * np.sin(y))
    y_m = params.a_m * y_m + params.b_m * r
    if k > 10000:
        max_err_late = max(max_err_late, abs(y - y_m))

w_norm = c.output_weight_norm()
print(f"Final y={y:.4f} y_m={y_m:.4f} late |y-y_m|max={max_err_late:.4f} weightNorm={w_norm:.3f}")

ok = np.isfinite(y) and np.isfinite(w_norm) and max_err_late < 0.15 and w_norm < 1e3
print("[PASS] All checks passed." if ok else "[FAIL] One or more checks failed.")
sys.exit(0 if ok else 1)
