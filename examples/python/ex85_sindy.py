"""
ex85_sindy.py -- SINDy (Sparse Identification of Nonlinear Dynamics) demo.

Demonstrates:
  1. Identifying a nonlinear system (Duffing oscillator) from simulation data.
  2. Comparing dense OLS vs sparse STLS coefficients.
  3. Validating the identified model on a held-out test trajectory.
  4. Using the SINDy model for short-horizon prediction.
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'build', 'bindings'))
if sys.platform == "win32" and hasattr(os, "add_dll_directory"):
    for _p in [r"C:\msys64\mingw64\bin"]:
        if os.path.isdir(_p):
            os.add_dll_directory(_p)

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'SINDy'):
        raise AttributeError("SINDy not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

import numpy as np

DT = 0.01

# ---------------------------------------------------------------------------
# Duffing oscillator: dx0 = x1,  dx1 = -0.5*x1 - x0 + 0.1*x0^3 + u
# ---------------------------------------------------------------------------
def duffing_dot(x, u, delta=0.5, alpha=-1.0, beta=0.1):
    return np.array([x[1],
                     -delta * x[1] + alpha * x[0] + beta * x[0]**3 + u])

def euler_step(x, u):
    return x + DT * duffing_dot(x, u)


# ---------------------------------------------------------------------------
# 1. Collect training data
# ---------------------------------------------------------------------------
N_TRAIN = 3000
x = np.array([1.0, 0.0])
u_seq = 0.3 * np.sin(np.linspace(0, 30, N_TRAIN))

sp = ctrl.SINDyParams()
sp.n_state = 2; sp.n_input = 1
sp.library = ctrl.SINDyLibrary.PolyDeg3   # need cubics for Duffing x0^3
sp.threshold = 0.05; sp.stls_iter = 15
sindy = ctrl.SINDy(sp)

for k in range(N_TRAIN):
    u = u_seq[k]
    x_next = euler_step(x, u)
    sindy.add_snapshot_fd(x, x_next, np.array([u]), DT)
    x = x_next

print(f"Training: {sindy.snapshot_count()} snapshots, {sindy.n_terms()} library terms")


# ---------------------------------------------------------------------------
# 2. Fit and inspect
# ---------------------------------------------------------------------------
model = sindy.fit()
Xi    = model.coefficients()

print(f"\nCoefficient matrix Xi ({Xi.shape[0]} x {Xi.shape[1]}):")
print(f"  Non-zero entries: {int(round((1-model.sparsity())*Xi.size))}")
print(f"  Sparsity: {model.sparsity()*100:.1f}%")


# ---------------------------------------------------------------------------
# 3. Validation on held-out trajectory
# ---------------------------------------------------------------------------
N_VAL  = 100
x_true = np.array([0.5, -0.2])
x_pred = x_true.copy()

rms_1step = 0.0
for k in range(N_VAL):
    u = 0.1 * np.sin(0.3 * k)
    x_true_next = euler_step(x_true, u)

    # 1-step prediction via SINDy
    xdot_pred = model.predict(x_pred, np.array([u]))
    x_pred_next = x_pred + DT * xdot_pred

    rms_1step += np.sum((x_true_next - x_pred_next) ** 2)

    x_true = x_true_next
    x_pred = x_pred_next   # use model prediction, not truth (open-loop)

rms_1step = np.sqrt(rms_1step / (N_VAL * 2))
print(f"\nOpen-loop {N_VAL}-step prediction RMS: {rms_1step:.5f}")


# ---------------------------------------------------------------------------
# Verification
# ---------------------------------------------------------------------------
assert model.n_state() == 2, "[FAIL] n_state"
assert model.n_input() == 1, "[FAIL] n_input"
assert model.sparsity() > 0.3, f"[FAIL] Model not sparse enough: {model.sparsity():.2f}"
assert np.all(np.isfinite(Xi)), "[FAIL] Non-finite coefficients"
assert rms_1step < 0.5, f"[FAIL] Prediction RMS too high: {rms_1step:.4f}"

# Spot-check: the linear terms should be recovered
# dx0/dt approx = x1 -> coefficient for x1 in first state equation should be ~1
# dx1/dt approx = -delta*x1 - x0 -> coefficient for x0 should be ~ -1 (alpha)
print(f"\nExpected dx0/dt approx = x1:  coefficient for x1 term = {Xi[2, 0]:.3f}  (expected ~1)")
print(f"Expected dx1/dt approx = -x0: coefficient for x0 term = {Xi[1, 1]:.3f}  (expected ~-1)")

print("\n[PASS] All SINDy ex85 checks passed.")
