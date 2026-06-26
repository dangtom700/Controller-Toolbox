"""
ex128_narmax.py

Phase 3 (SI4): polynomial NARMAX identification via orthogonal forward regression.

Mirrors ex111_narmax.cpp - fits a known bilinear NARX system
y[k] = 0.5 y[k-1] + 0.3 u[k-1] + 0.2 y[k-1] u[k-1] and checks term recovery plus one-step
held-out prediction accuracy.
"""

import sys
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'NARMAXIdentifier'):
        raise AttributeError("NARMAXIdentifier not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

rng = np.random.default_rng(12345)
N = 600
u = rng.uniform(-1.0, 1.0, N)
y = np.zeros(N)
for k in range(1, N):
    y[k] = 0.5 * y[k - 1] + 0.3 * u[k - 1] + 0.2 * y[k - 1] * u[k - 1]

N_fit = 500
params = ctrl.NARMAXParams()
params.na = 1
params.nb = 1
params.nc = 0
params.poly_degree = 2
params.significance_tol = 1e-4
params.max_terms = 6

res = ctrl.NARMAXIdentifier.fit(u[:N_fit], y[:N_fit], params)
print(f"Selected {len(res.selected_terms)} terms, cumulative ERR = {res.final_err_sum:.5f}:")
for term, coeff in zip(res.selected_terms, res.coefficients):
    print(f"  {term:<16s} coeff={coeff:.4f}")

terms_ok = all(t in res.selected_terms for t in ("y(k-1)", "u(k-1)", "y(k-1)*u(k-1)"))

sse, cnt = 0.0, 0
for k in range(N_fit, N):
    u_hist = np.array([u[k - 1]])
    y_hist = np.array([y[k - 1]])
    yhat = ctrl.NARMAXIdentifier.predict(res, u_hist, y_hist)
    sse += (yhat - y[k]) ** 2
    cnt += 1
rmse = np.sqrt(sse / cnt)
print(f"Held-out one-step RMSE = {rmse:.3e}")

ok = terms_ok and res.final_err_sum > 0.999 and rmse < 1e-6
print("[PASS] All checks passed." if ok else "[FAIL] One or more checks failed.")
sys.exit(0 if ok else 1)
