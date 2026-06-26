"""
ex125_neural_network_controller.py

Phase 3 (ML1): generic feedforward NeuralNetworkController forward pass.

Mirrors ex108_neural_network_controller.cpp - a single Linear layer realises a linear
state-feedback law u = -x1 - 2*x2 regulating a double integrator to the origin.
"""

import sys
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'NeuralNetworkController'):
        raise AttributeError("NeuralNetworkController not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

Ts = 0.01

layer = ctrl.NNLayerSpec()
layer.W = np.array([[-1.0, -2.0]])
layer.b = np.zeros(1)
layer.activation = ctrl.NNLayerSpec.Activation.Linear

params = ctrl.NeuralControllerParams()
params.layers = [layer]
params.n_input_features = 2
nn = ctrl.NeuralNetworkController(params, Ts)

x = np.array([1.0, 0.0])
for _ in range(4000):
    u = nn.compute_vec(x)[0]
    x[0] += Ts * x[1]
    x[1] += Ts * u

print(f"Regulated state: x1={x[0]:.5f} x2={x[1]:.5f}")

ok = np.all(np.isfinite(x)) and abs(x[0]) < 1e-2 and abs(x[1]) < 1e-2
print("[PASS] All checks passed." if ok else "[FAIL] One or more checks failed.")
sys.exit(0 if ok else 1)
