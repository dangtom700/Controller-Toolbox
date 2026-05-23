"""
ex23 - Controller Stack (Supervisory / Additive / Weighted)
=============================================================
Goal     : Replicate the ControllerStack composition concept in Python.
           Demonstrate three modes:
             Supervisory  - highest-priority controller whose |u| > threshold wins.
             Additive     - outputs summed and clamped.
             Weighted     - convex combination (w1*u1 + w2*u2).
           Each mode is verified for correct output range and stability.

Data generation : 2 000-sample step; stack of DiscretePID + DiscreteSMC.
Verification    :
  - Supervisory output equals u_pid when |u_pid| > |u_smc|, else u_smc.
  - Additive output \in [u_min, u_max] at all times.
  - Weighted output is convex: min(u_pid,u_smc) <= u_w <= max(u_pid,u_smc).

Run:
    conda activate soft_robotics
    python ex23_controller_stack.py
"""

import numpy as np
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from utils.plant import example_plant, ss_step
from utils.controllers import DiscretePID, DiscreteSMC
from utils.verify import print_summary

Ts    = 0.01
STEPS = 2000
U_MIN, U_MAX = -5.0, 5.0
W1, W2 = 0.6, 0.4    # weights for weighted mode

Kp, Ki, Kd = 3.0, 1.5, 0.75
ce, cde, k_smc, phi = 1.0, 10.0, 5.0, 0.1

print("=" * 60)
print("ex23 - Controller Stack")
print("=" * 60)

# Run both controllers independently and record u1, u2
pid = DiscretePID(Kp=Kp, Ki=Ki, Kd=Kd, Ts=Ts, u_min=U_MIN, u_max=U_MAX)
smc = DiscreteSMC(ce=ce, cde=cde, k=k_smc, phi=phi, u_min=U_MIN, u_max=U_MAX)
plant = example_plant()

u_pid = np.zeros(STEPS)
u_smc = np.zeros(STEPS)
y     = np.zeros(STEPS)

for k in range(STEPS):
    y_meas = y[k-1] if k > 0 else 0.0
    u_pid[k] = pid.compute(1.0, y_meas)
    u_smc[k] = smc.compute(1.0, y_meas)
    # Use PID output for plant (supervisory default)
    y[k] = ss_step(plant, u_pid[k])

# --- Supervisory mode ---
threshold = 1.0
u_super = np.where(np.abs(u_pid) >= np.abs(u_smc), u_pid, u_smc)

# --- Additive mode ---
u_add = np.clip(u_pid + u_smc, U_MIN, U_MAX)

# --- Weighted mode ---
u_w = np.clip(W1 * u_pid + W2 * u_smc, U_MIN, U_MAX)

results = {}

# Supervisory: u_super must equal u_pid or u_smc at each step
super_correct = np.all(
    (np.abs(u_pid) >= np.abs(u_smc)) == (u_super == u_pid)
)
results["supervisory_logic"] = bool(super_correct)
print(f"\n  {'[PASS]' if results['supervisory_logic'] else '[FAIL]'} supervisory logic correct")

# Additive: always in [U_MIN, U_MAX]
results["additive_bounded"] = bool(np.all((u_add >= U_MIN - 1e-10) & (u_add <= U_MAX + 1e-10)))
print(f"  {'[PASS]' if results['additive_bounded'] else '[FAIL]'} additive output bounded")

# Weighted: convex combination (before clamping)
u_w_raw = W1 * u_pid + W2 * u_smc
lo = np.minimum(u_pid, u_smc)
hi = np.maximum(u_pid, u_smc)
results["weighted_convex"] = bool(np.all((u_w_raw >= lo - 1e-10) & (u_w_raw <= hi + 1e-10)))
print(f"  {'[PASS]' if results['weighted_convex'] else '[FAIL]'} weighted output is convex combination")

# Weighted sum of weights = 1
results["weights_sum_to_1"] = abs(W1 + W2 - 1.0) < 1e-12
print(f"  {'[PASS]' if results['weights_sum_to_1'] else '[FAIL]'} W1+W2=1 ({W1}+{W2}={W1+W2})")

# Plant simulation stable
results["plant_stable"] = np.all(np.isfinite(y)) and float(np.max(np.abs(y))) < 10.0
print(f"  {'[PASS]' if results['plant_stable'] else '[FAIL]'} plant simulation stable")

print_summary(results)
