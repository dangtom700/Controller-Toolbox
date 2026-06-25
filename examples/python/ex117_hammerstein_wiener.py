"""
ex117_hammerstein_wiener.py

Phase 3 (SI5): Hammerstein identification of a valve-with-cubic-nonlinearity system.

Mirrors ex100_hammerstein_wiener.cpp - recovers both a static valve nonlinearity and the
linear actuator dynamics from input/output data alone.
"""

import sys
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'HammersteinWienerIdentifier'):
        raise AttributeError("HammersteinWienerIdentifier not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

rng = np.random.default_rng(5)
N = 400
u = rng.uniform(-1.0, 1.0, N)
v = u + 0.3 * u ** 3  # valve nonlinearity
y = np.zeros(N)
for k in range(1, N):
    y[k] = 0.8 * y[k - 1] + 0.5 * v[k - 1]  # linear actuator

params = ctrl.HammersteinWienerParams()
params.na, params.nb, params.nl_degree = 1, 1, 3
result = ctrl.HammersteinWienerIdentifier.fit_hammerstein(u, y, 0.1, params)

print(f"Recovered static nonlinearity coefficients [c0..c3]: {result.nl_input_coeffs} "
      f"(true: [0, 1, 0, 0.3])")
print(f"Recovered linear part: num={list(result.linear_part.num)}  "
      f"den={list(result.linear_part.den)} (true: num=[0, 0.5], den=[1, -0.8])")
print(f"Converged={result.converged} after {result.iters} iterations")

ok = (np.all(np.isfinite(result.nl_input_coeffs))
      and abs(result.nl_input_coeffs[1] - 1.0) < 1e-9
      and abs(result.nl_input_coeffs[3] - 0.3) < 0.05
      and abs(result.linear_part.den[1] - (-0.8)) < 0.05)
print("[PASS] All checks passed." if ok else "[FAIL] One or more checks failed.")
sys.exit(0 if ok else 1)
