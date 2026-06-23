"""
ex107_frequency_domain_identification.py

Phase 4 (Iteration 2): frequency-domain system identification (Levy's method).

Fits FreqDomainIdentifier.fit_levy() against the README's minimal-example plant's own
(noiseless) frequency response and confirms exact coefficient recovery.

    Plant: TransferFunction([0.0048, 0.0047], [1.0, -1.81, 0.819], Ts=0.01)

tf2ss() right-pads a numerator shorter than den+1 with leading zeros (so the README's
2-element numerator becomes the 3-element [0.0, 0.0048, 0.0047] once converted to
StateSpace) - the plant is written out in that explicit, unambiguous form below so the
assumed num_order matches what tf2ss actually realises.
"""

import sys
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'FreqDomainIdentifier'):
        raise AttributeError("FreqDomainIdentifier not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

Ts = 0.01
true_num = [0.0, 0.0048, 0.0047]
true_den = [1.0, -1.81, 0.819]

tf_true = ctrl.TransferFunction(true_num, true_den, Ts)
plant = ctrl.tf2ss(tf_true)

freqs = list(np.linspace(1.0, np.pi / Ts - 1.0, 60))
response = ctrl.SystemAnalysis.get_frequency_response(plant, freqs)

result = ctrl.FreqDomainIdentifier.fit_levy(freqs, response, num_order=2, den_order=2, Ts=Ts)

ok = True
if not result.full_rank:
    print("[FAIL] linear system was rank-deficient")
    ok = False

num_err = max(abs(a - b) for a, b in zip(result.tf.num, true_num))
den_err = max(abs(a - b) for a, b in zip(result.tf.den, true_den))
print(f"  recovered num: {list(result.tf.num)} (max error {num_err:.2e})")
print(f"  recovered den: {list(result.tf.den)} (max error {den_err:.2e})")
print(f"  rmse: {result.rmse:.2e}")

if num_err > 1e-6:
    print("[FAIL] numerator coefficients did not recover to tolerance")
    ok = False
if den_err > 1e-6:
    print("[FAIL] denominator coefficients did not recover to tolerance")
    ok = False
if result.rmse > 1e-6:
    print("[FAIL] fit RMSE too large for noiseless exact-order data")
    ok = False

print("[PASS] All checks passed." if ok else "[FAIL] One or more checks failed.")
sys.exit(0 if ok else 1)
