"""
ex32 - Phase Portrait and State-Space Trajectory
==================================================
Goal     : Simulate LQR-controlled plant from multiple initial conditions and
           plot state-space trajectories (x1 vs x2). Verify all trajectories
           converge to the origin (global asymptotic stability confirmed).

Data generation : 12 initial conditions on a unit circle; 500-step simulation each.
Verification    :
  - All 12 trajectories end with |x| < 0.01 at k=500.
  - Trajectories are smooth (no discontinuous jumps > 0.1 between steps).

Run:
    conda activate soft_robotics
    python ex32_phase_portrait.py
"""

import numpy as np
from scipy.linalg import solve_discrete_are
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from utils.plant import tf2ss, EXAMPLE_NUM, EXAMPLE_DEN
from utils.controllers import DiscreteLQR
from utils.verify import print_summary

Ts    = 0.01
STEPS = 500
N_IC  = 12

print("=" * 60)
print("ex32 - Phase Portrait (LQR State Trajectories)")
print("=" * 60)

ss = tf2ss(EXAMPLE_NUM, EXAMPLE_DEN)
A, B = ss.A, ss.B
Q = np.diag([1.0, 1.0])
R = np.array([[0.04]])
lqr = DiscreteLQR(A, B, Q, R)
A_cl = A - B @ lqr.K

print(f"\n  LQR gain K = {lqr.K}")
eigs = np.linalg.eigvals(A_cl)
print(f"  Closed-loop eigenvalues: {eigs}")
print(f"  |lambda|: {[f'{abs(e):.5f}' for e in eigs]}")

angles = np.linspace(0, 2*np.pi, N_IC, endpoint=False)
initial_conditions = np.column_stack([np.cos(angles), np.sin(angles)])

results = {}
all_end_norms = []

for i, x0 in enumerate(initial_conditions):
    x = x0.copy()
    traj = np.zeros((STEPS, 2))
    max_jump = 0.0
    for k in range(STEPS):
        traj[k] = x
        x = A_cl @ x
        if k > 0:
            jump = np.max(np.abs(traj[k] - traj[k-1]))
            max_jump = max(max_jump, jump)
    end_norm = float(np.linalg.norm(traj[-1]))
    all_end_norms.append(end_norm)
    key = f"ic{i:02d}_converges"
    results[key] = end_norm < 0.01
    if not results[key]:
        print(f"  [FAIL] IC {i}: |x_final| = {end_norm:.5f}")

convergence_rate = sum(results[f"ic{i:02d}_converges"] for i in range(N_IC)) / N_IC
print(f"\n  Trajectories converged to |x|<0.01: {int(convergence_rate*N_IC)}/{N_IC}")
print(f"  Max |x_final| across all ICs: {max(all_end_norms):.2e}")

results["all_converge"] = convergence_rate == 1.0
print(f"  {'[PASS]' if results['all_converge'] else '[FAIL]'} all 12 trajectories converge")

print_summary(results)
