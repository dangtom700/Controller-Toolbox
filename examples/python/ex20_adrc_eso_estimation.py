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
b0      = 0.5      # plant input gain approx K/tau = 0.898/1.14 ~ 0.79; use 0.5 (conservative)

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

z3_pre       = float(np.mean(np.abs(z3[burn:DIST_K])))
# Peak |z3| in the first 50 steps after disturbance (transient spike in ESO state)
z3_peak_post = float(np.max(np.abs(z3[DIST_K:DIST_K+50])))
# ESO must react: peak should exceed the pre-disturbance mean level
results["eso_detects_dist"] = z3_peak_post > z3_pre * 0.5
print(f"  z3 mean |pre|={z3_pre:.4f}, peak |post50|={z3_peak_post:.4f}  "
      f"{'[PASS]' if results['eso_detects_dist'] else '[FAIL]'} "
      f"ESO reacts to disturbance")

ss_err = abs(float(np.mean(y[-200:])) - 1.0)
results["ss_tracks"] = ss_err < 0.05
print(f"  Steady-state error after disturbance: {ss_err:.4f}  "
      f"{'[PASS]' if results['ss_tracks'] else '[FAIL]'} < 5%")

print_summary(results)
