"""
ex111_sk_complex_fit.py

Phase 3 (FD1): SK-reweighted complex-response fitting vs. a one-shot Levy fit.

Mirrors ex94_sk_complex_fit.cpp - fits a lightly-damped 2nd-order resonance from a noisy
frequency-response sample set, comparing FreqDomainIdentifier.fit_levy() (one-shot) against
SKFit.fit_sk() (iteratively reweighted) on the same data.
"""

import sys
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'SKFit'):
        raise AttributeError("SKFit not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

Ts = 0.1
r, theta = 0.97, 0.6
a1 = -2.0 * r * np.cos(theta)
a2 = r * r
tf_true = ctrl.TransferFunction([0.0, 1.0 - r, 0.0], [1.0, a1, a2], Ts)
sys_ss = ctrl.tf2ss(tf_true)

freqs = list(0.5 * np.arange(1, 41))
response = np.array(ctrl.SystemAnalysis.get_frequency_response(sys_ss, freqs))

rng = np.random.default_rng(7)
response = response + (rng.normal(0.0, 0.01, len(response)) + 1j * rng.normal(0.0, 0.01, len(response)))

levy_result = ctrl.FreqDomainIdentifier.fit_levy(freqs, list(response), num_order=2, den_order=2, Ts=Ts)
sk_result = ctrl.SKFit.fit_sk(freqs, list(response), num_order=2, den_order=2, Ts=Ts)

print(f"Levy (one-shot)   rmse = {levy_result.rmse:.5f}")
print(f"SK   (reweighted) rmse = {sk_result.iter_cost[-1]:.5f} after "
      f"{len(sk_result.iter_cost)} iterations (converged={sk_result.converged})")
print("Per-iteration RMSE:", [f"{c:.5f}" for c in sk_result.iter_cost])

ok = np.isfinite(sk_result.iter_cost[-1]) and sk_result.iter_cost[-1] < levy_result.rmse
print("[PASS] All checks passed." if ok else "[FAIL] One or more checks failed.")
sys.exit(0 if ok else 1)
