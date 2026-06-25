"""
ex116_clf_controller.py

Phase 3 (NC4): CLF synthesis via Sontag's universal formula.

Mirrors ex99_clf_controller.cpp - stabilizes xdot = x^3 + u toward the origin using a
candidate CLF V(x) = x^2, comparing against the hand-derived Sontag closed form.
"""

import sys
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'CLFController'):
        raise AttributeError("CLFController not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

Ts = 0.01
params = ctrl.CLFParams()
params.alpha = 2.0
clf = ctrl.CLFController(
    lambda x: float(x[0] ** 2),
    lambda x: 2.0 * float(x[0]) ** 4,
    lambda x: 2.0 * float(x[0]),
    params, Ts)

x0 = 1.5
clf.set_state(np.array([x0]))
u0 = clf.compute(0.0)
a0 = 2.0 * x0 ** 4 + params.alpha * x0 ** 2
b0 = 2.0 * x0
u_expected = -(a0 + np.sqrt(a0 ** 2 + b0 ** 4)) / b0
print(f"Sontag formula check: u={u0:.6f}  expected={u_expected:.6f}")

x = x0
for _ in range(2000):
    clf.set_state(np.array([x]))
    u = clf.compute(0.0)
    x += Ts * (x ** 3 + u)
print(f"Final state after closed-loop simulation: x={x:.6f}")

ok = np.isfinite(u0) and abs(u0 - u_expected) < 1e-9 and np.isfinite(x) and abs(x) < 0.05
print("[PASS] All checks passed." if ok else "[FAIL] One or more checks failed.")
sys.exit(0 if ok else 1)
