"""
ex81_lpv_crossval.py
---------------------
Part 23: Full LPV identification cross-validation.

Tests ctrl.identify_lpv against a known affine LPV model:
  A(p) = A0 + A1*p,   B(p) = B0 + B1*p
  with p drawn uniformly from [0, 1].

Procedure:
  1. Generate noise-free trajectory from the true LPV model.
  2. Call ctrl.identify_lpv(X, U, Y, sched, degree=1, Ts).
  3. Compare identified A0, A1, B0, B1 coefficients to ground truth.
  4. Simulate identified model on a new trajectory and compare RMS error.

Acceptance:
  - Coefficient error < 5e-2 for A and B matrices (noise-free data).
  - Simulation RMS error < 1e-4 (identified model reproduces true trajectory).

Expected output: PASS
"""
import sys
import numpy as np

sys.path.insert(0, __file__.rsplit("\\", 1)[0])
import _setup_bindings
try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'identify_lpv'):
        raise AttributeError("identify_lpv not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

# -----------------------------------------------------------------------
# True LPV model parameters
# -----------------------------------------------------------------------
# A(p) = [[0.9 - 0.2*p,  0.05],
#          [0.0,          0.8 + 0.1*p]]
# B(p) = [[0.1 + 0.1*p],
#          [0.2         ]]
# State dim n=2, input dim m=1

A0_true = np.array([[0.9,  0.05],
                     [0.0,  0.8 ]])
A1_true = np.array([[-0.2, 0.0 ],
                     [ 0.0, 0.1 ]])
B0_true = np.array([[0.1],
                     [0.2]])
B1_true = np.array([[0.1],
                     [0.0]])
C_true  = np.array([[1.0, 0.0]])  # y = x1

Ts = 0.1
n, m, p = 2, 1, 1

rng = np.random.default_rng(42)

# -----------------------------------------------------------------------
# Generate identification trajectory (noise-free)
# -----------------------------------------------------------------------
N_id = 500
sched_id = rng.uniform(0.0, 1.0, N_id)   # random scheduling sequence
u_id     = rng.standard_normal((N_id, m)) * 0.5

# ctrl.identify_lpv expects column-major layout: each column is one time step.
# X: (n x N+1),  U: (m x N),  Y: (p x N)
X_id = np.zeros((n, N_id + 1))
Y_id = np.zeros((p, N_id))
X_id[:, 0] = rng.standard_normal(n) * 0.1

for k in range(N_id):
    pk = sched_id[k]
    Ak = A0_true + A1_true * pk
    Bk = B0_true + B1_true * pk
    X_id[:, k + 1] = Ak @ X_id[:, k] + (Bk @ u_id[k]).flatten()
    Y_id[:, k] = (C_true @ X_id[:, k]).flatten()

# U must be (m x N) as well
# identify_lpv uses X[:,k] as source and X[:,k+1] as target for k=0..N-2.
# Pass all N_id states (x[0..N_id-1]) so we get N_id-1 regression pairs.
# x[N_id] is only needed as the last target - it's in X_id[:,N_id] but
# we include it by passing X_id unchanged (N_id+1 columns) and
# matching U and Y sizes with one padding column (set to zero, unused
# by the state regression; only affects output regression at step N_id).
N_cols = N_id  # pass N_id samples; T_x = N_id-1 state pairs, N_id output pairs
X_ctrl = X_id[:, :N_cols]       # (n x N_id) = x[0..N_id-1]
U_ctrl = u_id.T[:, :N_cols]     # (m x N_id)
Y_ctrl = Y_id[:, :N_cols]       # (p x N_id)

lpv_model = ctrl.identify_lpv(X_ctrl, U_ctrl, Y_ctrl, sched_id, 1, Ts)

# -----------------------------------------------------------------------
# Extract identified coefficients
# -----------------------------------------------------------------------
# LPVModel stores A_coeffs as a list of matrices: [A0, A1, ..., A_degree]
# Similarly for B_coeffs
A_coeffs = lpv_model.A_coeffs   # should be a list of 2 matrices (degree+1)
B_coeffs = lpv_model.B_coeffs

A0_id = np.array(A_coeffs[0])
A1_id = np.array(A_coeffs[1])
B0_id = np.array(B_coeffs[0])
B1_id = np.array(B_coeffs[1])

err_A0 = np.max(np.abs(A0_id - A0_true))
err_A1 = np.max(np.abs(A1_id - A1_true))
err_B0 = np.max(np.abs(B0_id - B0_true))
err_B1 = np.max(np.abs(B1_id - B1_true))

print(f"Coefficient errors: A0={err_A0:.2e}  A1={err_A1:.2e}  B0={err_B0:.2e}  B1={err_B1:.2e}")

# -----------------------------------------------------------------------
# Cross-validation on a fresh trajectory
# -----------------------------------------------------------------------
N_val = 200
sched_val = rng.uniform(0.0, 1.0, N_val)
u_val     = rng.standard_normal((N_val, m)) * 0.5

u_val_T = u_val.T   # (m x N_val) column-major for consistency

x_true  = rng.standard_normal(n) * 0.1
x_id    = x_true.copy()

rms_err = 0.0
for k in range(N_val):
    pk = sched_val[k]

    # True plant step
    Ak_t = A0_true + A1_true * pk
    Bk_t = B0_true + B1_true * pk
    x_true = Ak_t @ x_true + (Bk_t @ u_val_T[:, k]).flatten()

    # Identified model step (via frozen(p))
    ss_id = lpv_model.frozen(pk)
    A_f = np.array(ss_id.A)
    B_f = np.array(ss_id.B)
    x_id = A_f @ x_id + (B_f @ u_val_T[:, k]).flatten()

    rms_err += np.sum((x_true - x_id) ** 2)

rms_err = np.sqrt(rms_err / (N_val * n))
print(f"Cross-validation state RMS error: {rms_err:.2e}")

# -----------------------------------------------------------------------
# Acceptance
# -----------------------------------------------------------------------
coeff_ok = (err_A0 < 5e-2 and err_A1 < 5e-2 and
            err_B0 < 5e-2 and err_B1 < 5e-2)
sim_ok   = rms_err < 1e-4

if coeff_ok and sim_ok:
    print("PASS")
else:
    if not coeff_ok:
        print(f"FAIL: coefficient error too large")
    if not sim_ok:
        print(f"FAIL: simulation RMS {rms_err:.2e} >= 1e-4")
    sys.exit(1)
