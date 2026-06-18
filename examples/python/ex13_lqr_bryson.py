"""
ex13 - LQR Design via Bryson's Rule
=====================================
Goal     : Apply Bryson's Rule to set LQR weights Q and R, compute the DARE
           gain, close the loop with full-state feedback, and verify that the
           closed-loop poles are inside the unit disk.

Data generation : State trajectories from closed-loop simulation (2 000 steps).
Verification    :
  - All closed-loop eigenvalues have |lambda| < 1.
  - State converges to zero from a non-zero initial condition.
  - Q_ii = 1/xmax_i^2, R_jj = 1/umax_j^2 (Bryson's Rule check).

Run:
    conda activate soft_robotics
    python ex13_lqr_bryson.py
"""

import numpy as np
from scipy.linalg import solve_discrete_are
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from utils.plant import tf2ss, EXAMPLE_NUM, EXAMPLE_DEN
from utils.controllers import DiscreteLQR
from utils.verify import print_summary, assert_close

Ts    = 0.01
STEPS = 2000

print("=" * 60)
print("ex13 - LQR via Bryson's Rule")
print("=" * 60)

ss = tf2ss(EXAMPLE_NUM, EXAMPLE_DEN)
A, B, C, D = ss.A, ss.B, ss.C, ss.D

# Bryson's Rule: normalise by max allowable state and control values
x_max = np.array([1.0, 1.0])    # x1_max, x2_max
u_max_val = 5.0

Q = np.diag(1.0 / x_max**2)
R = np.array([[1.0 / u_max_val**2]])

print(f"\n  Q = diag({np.diag(Q)})")
print(f"  R = {R}")

results = {}
results["Q_bryson"] = assert_close(Q[0,0], 1.0/x_max[0]**2, label="Q[0,0]=1/xmax0^2")
results["R_bryson"] = assert_close(R[0,0], 1.0/u_max_val**2, label="R[0,0]=1/umax^2")

# Solve DARE
P = solve_discrete_are(A, B, Q, R)
K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
print(f"\n  LQR gain K = {K}")

# Closed-loop matrix and eigenvalues
A_cl = A - B @ K
eigs = np.linalg.eigvals(A_cl)
print(f"\n  Closed-loop eigenvalues:")
for i, ev in enumerate(eigs):
    mag = abs(ev)
    print(f"    lambda{i+1} = {ev:.6f},  |lambda{i+1}| = {mag:.6f}")
    results[f"eig{i+1}_stable"] = mag < 1.0

# Simulate regulation from x0 = [0.5, 0.1]
lqr = DiscreteLQR(A, B, Q, R)
x = np.array([0.5, 0.1])
ss.x = x.copy()

x_traj = np.zeros((STEPS, 2))
u_traj = np.zeros(STEPS)
for k in range(STEPS):
    x_traj[k] = ss.x
    u_traj[k] = lqr.compute(ss.x)
    # State update (no plant output needed here - state is observed directly)
    ss.x = A_cl @ ss.x

x_final = x_traj[-1]
print(f"\n  x initial = [0.5, 0.1]")
print(f"  x final   = [{x_final[0]:.2e}, {x_final[1]:.2e}]")

results["state_converges"] = float(np.max(np.abs(x_final))) < 1e-3
print(f"  {'[PASS]' if results['state_converges'] else '[FAIL]'} |x_final| < 1e-3")

print_summary(results)
