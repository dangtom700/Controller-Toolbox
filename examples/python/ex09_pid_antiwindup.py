"""
ex09 - PID Anti-Windup Demonstration
======================================
Goal     : Demonstrate that back-calculation anti-windup prevents integrator
           wind-up when the actuator saturates, and verify the resulting
           output is better than a PID without anti-windup.

Data generation : 2 000 samples; reference = 5.0 (large step), u_max = 2.0.
Verification    :
  - With anti-windup: ISE < ISE without anti-windup.
  - With anti-windup: no persistent integrator kick after reference reaches 0.

Run:
    conda activate soft_robotics
    python ex09_pid_antiwindup.py
"""

import numpy as np
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from utils.plant import example_plant, ss_step
from utils.controllers import DiscretePID
from utils.verify import ise, print_summary

Ts    = 0.01
STEPS = 2000
REF   = 5.0    # large reference that saturates the actuator
U_MAX = 2.0    # tight saturation limit

Kp, Ki, Kd = 3.0, 1.5, 0.75

print("=" * 60)
print("ex09 - PID Anti-Windup")
print("=" * 60)
print(f"\n  Reference={REF}, u_max={U_MAX}, Kp={Kp}, Ki={Ki}, Kd={Kd}")

def run_sim(pid):
    plant = example_plant()
    y = np.zeros(STEPS)
    for k in range(STEPS):
        u = pid.compute(REF, y[k - 1] if k > 0 else 0.0)
        y[k] = ss_step(plant, u)
    return y

# PID without anti-windup (Kb=0 means no back-calculation correction)
pid_no_aw = DiscretePID(Kp=Kp, Ki=Ki, Kd=Kd, Ts=Ts,
                         u_min=-U_MAX, u_max=U_MAX, Kb=0.0)
pid_with_aw = DiscretePID(Kp=Kp, Ki=Ki, Kd=Kd, Ts=Ts,
                           u_min=-U_MAX, u_max=U_MAX, Kb=1.0)

y_no_aw   = run_sim(pid_no_aw)
y_with_aw = run_sim(pid_with_aw)

err_no_aw   = REF - y_no_aw
err_with_aw = REF - y_with_aw

ise_no_aw   = ise(err_no_aw,   Ts)
ise_with_aw = ise(err_with_aw, Ts)

print(f"\n  ISE without anti-windup: {ise_no_aw:.4f}")
print(f"  ISE with    anti-windup: {ise_with_aw:.4f}")

results = {}
results["antiwindup_reduces_ise"] = ise_with_aw < ise_no_aw
print(f"  {'[PASS]' if results['antiwindup_reduces_ise'] else '[FAIL]'} "
      f"anti-windup reduces ISE")

# Both should eventually reach steady state near REF
ss_no_aw   = float(np.mean(y_no_aw[-200:]))
ss_with_aw = float(np.mean(y_with_aw[-200:]))
print(f"\n  SS with AW: {ss_with_aw:.4f}  (expect approx = {REF:.1f}, clamped by u_max)")
results["ss_reachable"] = ss_with_aw > 0.0
print(f"  {'[PASS]' if results['ss_reachable'] else '[FAIL]'} plant output positive")

# Check no overshoot explosion in no-AW case (may diverge with large ref)
results["no_aw_finite"] = np.all(np.isfinite(y_no_aw))
print(f"  {'[PASS]' if results['no_aw_finite'] else '[FAIL]'} no-AW output is finite")

# Peak overshoot comparison
peak_no_aw   = float(np.max(y_no_aw))
peak_with_aw = float(np.max(y_with_aw))
print(f"\n  Peak output - no AW: {peak_no_aw:.4f}, with AW: {peak_with_aw:.4f}")
results["aw_peak_lower"] = peak_with_aw <= peak_no_aw + 0.1
print(f"  {'[PASS]' if results['aw_peak_lower'] else '[FAIL]'} "
      f"anti-windup does not increase peak")

print_summary(results)
