"""
ex119_mle_identification.py

Phase 3 Roadmap Phase 2 (SI1): MLE/MAP identification on outlier-heavy data.

Mirrors ex102_mle_identification.cpp - a Laplace noise model is far less sensitive to
occasional quantizer-glitch outliers than the Gaussian-MLE (= least squares) fit.
"""

import sys
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'MLEIdentifier'):
        raise AttributeError("MLEIdentifier not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

rng = np.random.default_rng(3)
N = 300
true_a1, true_b1 = -0.6, 0.4  # y[k] = 0.6*y[k-1] + 0.4*u[k-1] + noise
u = rng.uniform(-1.0, 1.0, N)
y = np.zeros(N)
for k in range(1, N):
    noise = rng.uniform(-0.01, 0.01)
    if rng.uniform() < 0.05:
        noise += rng.choice([-5.0, 5.0])  # quantizer glitch
    y[k] = -true_a1 * y[k - 1] + true_b1 * u[k - 1] + noise

gauss_params = ctrl.MLEParams()
gauss_params.na, gauss_params.nb = 1, 1
gauss_params.noise = ctrl.NoiseModel.Gaussian
gauss_result = ctrl.MLEIdentifier.fit(u, y, 0.1, gauss_params)

laplace_params = ctrl.MLEParams()
laplace_params.na, laplace_params.nb = 1, 1
laplace_params.noise = ctrl.NoiseModel.Laplace
laplace_result = ctrl.MLEIdentifier.fit(u, y, 0.1, laplace_params)

true_theta = np.array([true_a1, true_b1])
print(f"True theta:   {true_theta}")
print(f"Gaussian MLE: {gauss_result.theta}")
print(f"Laplace MLE:  {laplace_result.theta}")

gauss_err = np.linalg.norm(gauss_result.theta - true_theta)
laplace_err = np.linalg.norm(laplace_result.theta - true_theta)
print(f"Gaussian error: {gauss_err:.4f}  Laplace error: {laplace_err:.4f}")

ok = laplace_err < gauss_err
print("[PASS] All checks passed." if ok else "[FAIL] One or more checks failed.")
sys.exit(0 if ok else 1)
