"""
ex20 - ADRC Extended State Observer Estimation
================================================
Goal     : Demonstrate the ADRC ESO tracking the plant output and the
           lumped disturbance z3 (generalised disturbance). Apply an external
           additive disturbance at k=1000 and verify the ESO detects it.

Data generation : 2 000-sample step + disturbance (+0.5 at k=1000).
Verification    :
  - ESO z1 tracks plant output: RMSE(z1, y) < 0.05 after burn-in.
  - z3 (disturbance estimate) spikes near k=1000 when disturbance is applied.
  - Closed-loop tracks reference despite disturbance (|y_ss - 1.0| < 2%).

Run:
    conda activate soft_robotics
    python ex20_adrc_eso_estimation.py
"""

import numpy as np
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from utils.plant import example_plant, ss_step
from utils.controllers import DiscreteADRC
from utils.verify import rmse, print_summary

Ts    = 0.01
STEPS = 2000
DIST_K = 1000
DIST_MAG = 0.5

omega_o = 20.0
omega_c = 4.0
b0      = 1.0e-4   # rough input gain: b0 approx = b1/Ts for 2nd order plant

print("=" * 60)
print("ex20 - ADRC / ESO Disturbance Estimation")
print("=" * 60)
print(f"\n  omega_o={omega_o}, omega_c={omega_c}, b0={b0}")

adrc  = DiscreteADRC(omega_o=omega_o, omega_c=omega_c, b0=b0,
                     Ts=Ts, u_min=-10.0, u_max=10.0)
plant = example_plant()

y    = np.zeros(STEPS)
z1   = np.zeros(STEPS)   # ESO y-estimate
z3   = np.zeros(STEPS)   # ESO disturbance estimate

for k in range(STEPS):
    y_prev = y[k-1] if k > 0 else 0.0
    u = adrc.compute(1.0, y_prev)
    dist = DIST_MAG if k >= DIST_K else 0.0
    y[k] = ss_step(plant, u + dist)
    z1[k] = adrc._z[0]
    z3[k] = adrc._z[2]

results = {}
results["stable"] = np.all(np.isfinite(y)) and float(np.max(np.abs(y))) < 10.0
print(f"\n  {'[PASS]' if results['stable'] else '[FAIL]'} closed loop stable")

burn = 200
rmse_eso = rmse(y[burn:DIST_K], z1[burn:DIST_K])
results["eso_tracks"] = rmse_eso < 0.05
print(f"  ESO z1 RMSE before disturbance: {rmse_eso:.5f}  "
      f"{'[PASS]' if results['eso_tracks'] else '[FAIL]'} < 0.05")

z3_pre  = float(np.mean(np.abs(z3[burn:DIST_K])))
z3_post = float(np.mean(np.abs(z3[DIST_K:DIST_K+200])))
results["eso_detects_dist"] = z3_post > 2.0 * z3_pre
print(f"  z3 mean |pre|={z3_pre:.4f}, |post|={z3_post:.4f}  "
      f"{'[PASS]' if results['eso_detects_dist'] else '[FAIL]'} post > 2* pre")

ss_err = abs(float(np.mean(y[-200:])) - 1.0)
results["ss_tracks"] = ss_err < 0.02
print(f"  Steady-state error after disturbance: {ss_err:.4f}  "
      f"{'[PASS]' if results['ss_tracks'] else '[FAIL]'} < 2%")

print_summary(results)
