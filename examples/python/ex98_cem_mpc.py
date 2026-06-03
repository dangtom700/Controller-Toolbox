"""
ex98_cem_mpc.py -- Cross-Entropy Method MPC demo.

Demonstrates:
  1. CEM-MPC on a double integrator (position + velocity).
  2. Step reference tracking from x=0 to x=1.
  3. Comparison with a PID baseline.
  4. Input saturation to uMin=-2, uMax=2.

Plant: double integrator  x[k+1] = A*x[k] + B*u[k],  y = x[0] (position)
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'build', 'bindings'))
if sys.platform == "win32" and hasattr(os, "add_dll_directory"):
    for _p in [r"C:\msys64\mingw64\bin"]:
        if os.path.isdir(_p):
            os.add_dll_directory(_p)

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'CEMController'):
        raise AttributeError("CEMController not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

import numpy as np

print("=== ex98: CEM-MPC on Double Integrator ===")

TS = 0.05
N  = 120

# Double integrator matrices
A = np.array([[1.0, TS], [0.0, 1.0]])
B = np.array([[0.5 * TS * TS], [TS]])
C = np.array([[1.0, 0.0]])

# StateFunc for CEM: x_{k+1} = A*x + B*u
def f(x, u):
    return A @ x + B @ u

# ---------------------------------------------------------------------------
# CEM-MPC
# ---------------------------------------------------------------------------
print("\n-- CEM-MPC setup --")
cp = ctrl.CEMParams()
cp.Np         = 20
cp.N_samples  = 100
cp.n_iter     = 5
cp.elite_frac = 0.1
cp.Q          = 100.0
cp.R          = 0.1
cp.uMin       = -2.0
cp.uMax       =  2.0
cp.sigma_init = 1.0
cp.seed       = 42

cem = ctrl.CEMController(cp, f, C, TS)

r_ref = np.array([1.0])

x_cem = np.array([0.0, 0.0])
iae_cem = 0.0

print(f"  {'k':>5}  {'pos':>8}  {'vel':>8}  {'u':>8}")
for k in range(N):
    cem.set_state(x_cem)
    cem.set_reference(r_ref)
    u_vec = cem.compute_ref(x_cem, r_ref)
    u = float(u_vec[0])

    x_cem = A @ x_cem + B.flatten() * u
    iae_cem += abs(1.0 - x_cem[0]) * TS

    if k in (0, 10, 20, 40, 60, 80, 100, 119):
        print(f"  {k:5d}  {x_cem[0]:8.4f}  {x_cem[1]:8.4f}  {u:8.4f}")

print(f"\n  CEM-MPC IAE: {iae_cem:.4f}")
print(f"  Final position: {x_cem[0]:.4f}  (target: 1.0)")

# ---------------------------------------------------------------------------
# PID baseline
# ---------------------------------------------------------------------------
print("\n-- PID baseline comparison --")
pid = ctrl.DiscretePID(10.0, 2.0, 0.5, TS, 1.0)

x_pid = np.array([0.0, 0.0])
iae_pid = 0.0
for k in range(N):
    e = 1.0 - x_pid[0]
    u = np.clip(pid.compute(e), -2.0, 2.0)
    x_pid = A @ x_pid + B.flatten() * u
    iae_pid += abs(1.0 - x_pid[0]) * TS

print(f"  PID IAE:     {iae_pid:.4f}")
print(f"  CEM-MPC IAE: {iae_cem:.4f}")
print(f"\n  Final position (PID):     {x_pid[0]:.4f}")
print(f"  Final position (CEM-MPC): {x_cem[0]:.4f}")

# ---------------------------------------------------------------------------
# Verify bounds
# ---------------------------------------------------------------------------
print("\n-- Input saturation verification --")
x_test = np.array([0.0, 0.0])
cem.set_state(x_test)
cem.set_reference(r_ref)
u_test = float(cem.compute_ref(x_test, r_ref)[0])
print(f"  u in [{cp.uMin}, {cp.uMax}]: {cp.uMin:.1f} <= {u_test:.4f} <= {cp.uMax:.1f}")
assert cp.uMin - 0.01 <= u_test <= cp.uMax + 0.01, "Input bounds violated"

print("\n[PASS] CEMController demo complete.")
