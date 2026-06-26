"""
ex131_complex_vector_fit.py

Phase 3 (FD2): complex-conjugate-pole Vector Fitting vs. a one-shot Levy fit.

Mirrors ex114_complex_vector_fit.cpp - fits a 3-resonance system (3 lightly-damped
complex-conjugate pole pairs) from a noisy frequency-response sample set with
ComplexVectorFit.fit(), comparing against FreqDomainIdentifier.fit_levy() (one-shot, same order)
on the same data.
"""

import sys
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'ComplexVectorFit'):
        raise AttributeError("ComplexVectorFit not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

Ts = 0.1
specs = [(0.99, 0.4), (0.985, 0.55), (0.99, 0.75)]


def poly_mul_pair(p, a1, a2):
    result = np.zeros(len(p) + 2)
    for i, c in enumerate(p):
        result[i] += c
        result[i + 1] += c * a1
        result[i + 2] += c * a2
    return result


den = np.array([1.0])
for r, theta in specs:
    den = poly_mul_pair(den, -2.0 * r * np.cos(theta), r * r)
num = np.zeros(len(den))
num[1] = 0.05

tf_true = ctrl.TransferFunction(list(num), list(den), Ts)
sys_ss = ctrl.tf2ss(tf_true)

freqs = list(0.25 * np.arange(1, 81))
response = np.array(ctrl.SystemAnalysis.get_frequency_response(sys_ss, freqs))

rng = np.random.default_rng(11)
response = response + (rng.normal(0.0, 0.02, len(response)) + 1j * rng.normal(0.0, 0.02, len(response)))

cvf_result = ctrl.ComplexVectorFit.fit(freqs, list(response), n_real_poles=0, n_complex_pairs=3,
                                        Ts=Ts, max_iter=30)
levy_result = ctrl.FreqDomainIdentifier.fit_levy(freqs, list(response), num_order=6, den_order=6, Ts=Ts)

print(f"Levy (one-shot, order 6)    rmse = {levy_result.rmse:.5f}")
print(f"ComplexVectorFit            rmse = {cvf_result.iter_error[-1]:.5f} after "
      f"{len(cvf_result.iter_error)} iterations (converged={cvf_result.converged})")
print("Recovered poles (magnitude @ angle [rad]):")
for p in cvf_result.poles:
    print(f"  {abs(p):.4f} @ {np.angle(p):.4f}")

ok = np.isfinite(cvf_result.iter_error[-1]) and cvf_result.iter_error[-1] < 0.5 * levy_result.rmse
print("[PASS] All checks passed." if ok else "[FAIL] One or more checks failed.")
sys.exit(0 if ok else 1)
