"""
ex19 - Sliding Mode Controller with Boundary Layer
====================================================
Goal     : Simulate DiscreteSMC and verify that the sliding surface s converges
           to the boundary layer |s| < φ within finite time (reaching phase),
           after which chattering is eliminated by the saturation function.

Data generation : 2 000-sample step response, SMC parameters ce=1, cde=10, k=5, phi=0.1.
Verification    :
  - |s| < φ for > 90% of steady-state samples (last 50%).
  - Chattering in u is quantified: std(u_ss) < std(u_reaching).
  - No divergence: |y| < 10.

Run:
    conda activate soft_robotics
    python ex19_smc_sliding_surface.py
"""

import numpy as np
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from utils.plant import example_plant, ss_step
from utils.controllers import DiscreteSMC
from utils.verify import print_summary

Ts    = 0.01
STEPS = 2000

ce, cde, k, phi = 1.0, 10.0, 5.0, 0.1

print("=" * 60)
print("ex19 - Sliding Mode Controller")
print("=" * 60)
print(f"\n  ce={ce}, cde={cde}, k={k}, phi={phi}")

smc   = DiscreteSMC(ce=ce, cde=cde, k=k, phi=phi, u_min=-10.0, u_max=10.0)
plant = example_plant()

y = np.zeros(STEPS)
u = np.zeros(STEPS)
s = np.zeros(STEPS)
e_prev = 0.0

for k_step in range(STEPS):
    e = 1.0 - (y[k_step-1] if k_step > 0 else 0.0)
    s[k_step] = ce * e + cde * (e - e_prev)
    u[k_step] = smc.compute(1.0, y[k_step-1] if k_step > 0 else 0.0)
    y[k_step] = ss_step(plant, u[k_step])
    e_prev = e

results = {}
results["stable"] = np.all(np.isfinite(y)) and float(np.max(np.abs(y))) < 10.0
print(f"\n  {'[PASS]' if results['stable'] else '[FAIL]'} y bounded (|y| < 10)")

# Reaching phase: first 30% of simulation
reach_end = STEPS // 3
ss_start  = STEPS // 2

s_reach = s[:reach_end]
s_ss    = s[ss_start:]
u_reach = u[:reach_end]
u_ss    = u[ss_start:]

in_boundary = float(np.mean(np.abs(s_ss) < phi))
results["in_boundary_layer"] = in_boundary > 0.90
print(f"  {'[PASS]' if results['in_boundary_layer'] else '[FAIL]'} "
      f"{in_boundary:.1%} of steady-state |s| < φ={phi}")

chatter_reach = float(np.std(u_reach))
chatter_ss    = float(np.std(u_ss))
results["chattering_reduced"] = chatter_ss < chatter_reach
print(f"  Chattering std - reaching: {chatter_reach:.4f}, steady-state: {chatter_ss:.4f}")
print(f"  {'[PASS]' if results['chattering_reduced'] else '[FAIL]'} "
      f"boundary layer reduces chattering")

ss_err = abs(float(np.mean(y[-200:])) - 1.0)
results["ss_error"] = ss_err < 0.05
print(f"  Steady-state error: {ss_err:.4f}  "
      f"{'[PASS]' if results['ss_error'] else '[FAIL]'}")

print_summary(results)
