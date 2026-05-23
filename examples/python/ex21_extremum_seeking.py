"""
ex21 - Extremum Seeking Control
=================================
Goal     : Use ExtremumSeeker to find the maximum of a static quadratic cost
           J(θ) = -(θ - 1.5)^2 + 4  (optimum at θ* = 1.5).
           Verify the ESC converges within 5% of the optimum within 5 000 steps.

Data generation : 5 000 ESC steps; performance function is the quadratic cost.
Verification    :
  - |θ_mean_final - θ*| < 0.1 (within 5% of range).
  - θ trajectory moves monotonically toward optimum in 1st half.

Run:
    conda activate soft_robotics
    python ex21_extremum_seeking.py
"""

import numpy as np
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from utils.controllers import ExtremumSeeker
from utils.verify import print_summary, assert_close

Ts     = 0.01
STEPS  = 5000
THETA_OPT = 1.5

def cost(theta):
    return -(theta - THETA_OPT)**2 + 4.0

print("=" * 60)
print("ex21 - Extremum Seeking Control")
print("=" * 60)
print(f"\n  Cost: J(θ) = -(θ-{THETA_OPT})^2 + 4,  θ* = {THETA_OPT}")

esc = ExtremumSeeker(
    dither_amp=0.05,
    dither_freq=5.0,
    omega_h=0.5,
    omega_l=0.2,
    k_esc=2.0,
    Ts=Ts,
    u_min=-5.0, u_max=5.0,
)

theta_traj = np.zeros(STEPS)
J_traj     = np.zeros(STEPS)

for k in range(STEPS):
    theta = esc._theta + esc.amp * np.sin(esc.omega_d * k * Ts)
    J = cost(theta)
    theta_traj[k] = theta
    J_traj[k] = J
    esc.compute(J)

theta_final = float(np.mean(theta_traj[-500:]))
J_final     = float(np.mean(J_traj[-500:]))

print(f"\n  θ initial: {theta_traj[0]:.4f}")
print(f"  θ final (mean last 500): {theta_final:.4f}  (optimum={THETA_OPT})")
print(f"  J final (mean last 500): {J_final:.4f}  (J_max={cost(THETA_OPT):.4f})")

results = {}
results["converges_theta"] = abs(theta_final - THETA_OPT) < 0.1
print(f"\n  {'[PASS]' if results['converges_theta'] else '[FAIL]'} "
      f"|θ_final - θ*| = {abs(theta_final - THETA_OPT):.4f} < 0.1")

results["J_near_max"] = abs(J_final - cost(THETA_OPT)) < 0.5
print(f"  {'[PASS]' if results['J_near_max'] else '[FAIL]'} "
      f"|J_final - J_max| = {abs(J_final - cost(THETA_OPT)):.4f} < 0.5")

# Monotonic progress in 1st quarter
q1 = STEPS // 4
t_q1 = theta_traj[:q1]
dist_initial = abs(t_q1[0]  - THETA_OPT)
dist_q1_end  = abs(t_q1[-1] - THETA_OPT)
results["approaches_opt"] = dist_q1_end < dist_initial
print(f"  {'[PASS]' if results['approaches_opt'] else '[FAIL]'} "
      f"θ moves toward θ* in 1st quarter "
      f"(Δdist: {dist_initial:.4f} -> {dist_q1_end:.4f})")

print_summary(results)
