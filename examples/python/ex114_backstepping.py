"""
ex114_backstepping.py

Phase 3 (NC1): Backstepping control of a 2-stage strict-feedback system.

Mirrors ex97_backstepping.cpp - x1' = x2, x2' = u, a relative-degree-2 structure that
FeedbackLinearisationController (relative-degree-1 only) cannot handle directly.
"""

import sys
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'BacksteppingController'):
        raise AttributeError("BacksteppingController not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

Ts = 0.01
params = ctrl.BacksteppingParams()
params.k_gains = [2.0, 2.0]
bc = ctrl.BacksteppingController(
    [lambda x, s: 0.0, lambda x, s: 0.0],
    [lambda x, s: 1.0, lambda x, s: 1.0],
    params, Ts)

ref = 1.0
x = np.zeros(2)
for _ in range(2000):
    bc.set_state(x)
    u = bc.compute(ref - x[0])
    x[0] += Ts * x[1]
    x[1] += Ts * u

print(f"Final state: x1={x[0]:.4f} (ref={ref})  x2={x[1]:.4f}")

ok = np.all(np.isfinite(x)) and abs(x[0] - ref) < 0.02 and abs(x[1]) < 0.05
print("[PASS] All checks passed." if ok else "[FAIL] One or more checks failed.")
sys.exit(0 if ok else 1)
