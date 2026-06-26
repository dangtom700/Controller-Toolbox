"""
ex127_nonlinear_imc.py

Phase 3 (NC3): Nonlinear Internal Model Control.

Mirrors ex110_nonlinear_imc.cpp - exact model match gives offset-free tracking (IMC's
defining property); a perturbed plant gain is still driven to zero steady-state offset by
the mismatch feedback path.
"""

import sys
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'NonlinearIMC'):
        raise AttributeError("NonlinearIMC not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)


def model_fn(x, u):
    return 0.7 * x[0] + 0.3 * u


def inverse_fn(x, y_t):
    return (y_t - 0.7 * x[0]) / 0.3


def run(plant_a, plant_b):
    Ts = 0.1
    params = ctrl.NonlinearIMCParams()
    params.filter_lambda = 0.5
    params.uMin = -100.0
    params.uMax = 100.0
    imc = ctrl.NonlinearIMC(model_fn, inverse_fn, params, Ts)

    r = 1.0
    y = 0.0
    for _ in range(500):
        x = np.array([y])
        imc.set_state(x)
        u = imc.compute(r - y)
        y = plant_a * y + plant_b * u
    return y


y_exact = run(0.7, 0.3)
y_mis = run(0.75, 0.28)
print(f"Exact-match y={y_exact:.5f}  Mismatch y={y_mis:.5f} (ref=1.0)")

ok = (np.isfinite(y_exact) and np.isfinite(y_mis) and
      abs(y_exact - 1.0) < 1e-3 and abs(y_mis - 1.0) < 1e-2)
print("[PASS] All checks passed." if ok else "[FAIL] One or more checks failed.")
sys.exit(0 if ok else 1)
