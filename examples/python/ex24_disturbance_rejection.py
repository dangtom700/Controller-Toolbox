"""
ex24 - Disturbance Rejection Comparison (PID vs ADRC)
=======================================================
Goal     : Apply a step load disturbance (+0.3 at k=1000) to both a PID and
           ADRC closed loop, record recovery time and ISE post-disturbance,
           and verify ADRC rejects faster (fewer samples to re-enter 2% band).

Data generation : 2 000 samples; reference = 1.0 throughout.
Verification    :
  - Both controllers return to +/-2% of reference after disturbance.
  - ADRC recovery time <= PID recovery time.

Run:
    conda activate soft_robotics
    python ex24_disturbance_rejection.py
"""

import numpy as np
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from utils.plant import example_plant, ss_step
from utils.controllers import DiscretePID, DiscreteADRC
from utils.verify import ise, print_summary

Ts     = 0.01
STEPS  = 2000
DIST_K = 1000
DIST   = 0.3

Kp, Ki, Kd = 3.0, 1.5, 0.75
omega_o, omega_c_adrc, b0 = 20.0, 4.0, 1e-4

print("=" * 60)
print("ex24 - Disturbance Rejection: PID vs ADRC")
print("=" * 60)
print(f"\n  Disturbance: +{DIST} at k={DIST_K}")

def sim_pid_dist():
    pid   = DiscretePID(Kp=Kp, Ki=Ki, Kd=Kd, Ts=Ts, u_min=-10.0, u_max=10.0)
    plant = example_plant()
    y = np.zeros(STEPS)
    for k in range(STEPS):
        u = pid.compute(1.0, y[k-1] if k > 0 else 0.0)
        dist = DIST if k >= DIST_K else 0.0
        y[k] = ss_step(plant, u + dist)
    return y

def sim_adrc_dist():
    adrc  = DiscreteADRC(omega_o=omega_o, omega_c=omega_c_adrc, b0=b0,
                         Ts=Ts, u_min=-10.0, u_max=10.0)
    plant = example_plant()
    y = np.zeros(STEPS)
    for k in range(STEPS):
        u = adrc.compute(1.0, y[k-1] if k > 0 else 0.0)
        dist = DIST if k >= DIST_K else 0.0
        y[k] = ss_step(plant, u + dist)
    return y

y_pid  = sim_pid_dist()
y_adrc = sim_adrc_dist()

def recovery_steps(y, ref=1.0, tol=0.02, from_step=DIST_K):
    """Steps from from_step until y stays within tol of ref."""
    in_band = np.abs(y[from_step:] - ref) <= tol
    for i, ok in enumerate(in_band):
        if ok and (i + from_step + 10 < STEPS) and np.all(in_band[i:i+10]):
            return i
    return STEPS - from_step

r_pid  = recovery_steps(y_pid)
r_adrc = recovery_steps(y_adrc)

ise_pid_post  = ise(1.0 - y_pid[DIST_K:],  Ts)
ise_adrc_post = ise(1.0 - y_adrc[DIST_K:], Ts)

print(f"\n  PID  recovery: {r_pid}  steps ({r_pid*Ts:.3f} s),  post-dist ISE: {ise_pid_post:.5f}")
print(f"  ADRC recovery: {r_adrc} steps ({r_adrc*Ts:.3f} s),  post-dist ISE: {ise_adrc_post:.5f}")

results = {}
results["pid_recovers"]  = r_pid  < STEPS - DIST_K
results["adrc_recovers"] = r_adrc < STEPS - DIST_K
print(f"\n  {'[PASS]' if results['pid_recovers']  else '[FAIL]'} PID recovers")
print(f"  {'[PASS]' if results['adrc_recovers'] else '[FAIL]'} ADRC recovers")

results["adrc_faster_or_equal"] = r_adrc <= r_pid
print(f"  {'[PASS]' if results['adrc_faster_or_equal'] else '[FAIL]'} "
      f"ADRC recovery <= PID recovery ({r_adrc} <= {r_pid})")

results["both_stable"] = (np.all(np.isfinite(y_pid)) and np.all(np.isfinite(y_adrc)))
print(f"  {'[PASS]' if results['both_stable'] else '[FAIL]'} both controllers stable")

print_summary(results)
