"""
ex09 - PID Anti-Windup Demonstration
======================================
Goal     : Demonstrate that back-calculation anti-windup prevents integrator
           wind-up when the actuator saturates, and verify the resulting
           output is better than a PID without anti-windup.

Data generation : 3 000 samples; two phases.
  Phase 1 (k=0..1499): REF=1.5 - unachievable with U_MAX=1.2 (max y~1.08),
    so the actuator saturates for the full 15 s and the integral winds up.
  Phase 2 (k=1500..2999): REF=0  - the wound-up integral in the no-AW
    controller prevents fast recovery, while the AW controller tracks cleanly.
Verification    :
  - With anti-windup: total ISE < ISE without anti-windup.
  - Both controllers remain stable (finite, bounded output).

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

Ts     = 0.01
STEPS  = 3000   # 30 s total
PHASE1 = 1500   # phase 1: k < 1500 -> REF_HI (saturating), phase 2: REF_LO
U_MAX  = 1.2    # tight saturation; max y = 1.2*0.898 = 1.077 < REF_HI -> always saturated
REF_HI = 1.5    # unachievable reference -> sustained saturation -> windup in no-AW
REF_LO = 0.0    # step to 0: reveals wound-up integral in no-AW case

Kp, Ki, Kd = 1.5, 3.0, 0.0   # high Ki amplifies the windup difference

print("=" * 60)
print("ex09 - PID Anti-Windup")
print("=" * 60)
print(f"\n  Phase-1 REF={REF_HI} (u_max={U_MAX} -> always saturated), "
      f"Phase-2 REF={REF_LO} (reveals windup), Kp={Kp}, Ki={Ki}, Kd={Kd}")

def run_sim(pid):
    plant = example_plant()
    y = np.zeros(STEPS)
    for k in range(STEPS):
        ref = REF_HI if k < PHASE1 else REF_LO
        u = pid.compute(ref, y[k - 1] if k > 0 else 0.0)
        y[k] = ss_step(plant, u)
    return y

pid_no_aw   = DiscretePID(Kp=Kp, Ki=Ki, Kd=Kd, Ts=Ts,
                           u_min=-U_MAX, u_max=U_MAX, Kb=0.0)
pid_with_aw = DiscretePID(Kp=Kp, Ki=Ki, Kd=Kd, Ts=Ts,
                           u_min=-U_MAX, u_max=U_MAX, Kb=1.0)

y_no_aw   = run_sim(pid_no_aw)
y_with_aw = run_sim(pid_with_aw)

# Reference trajectory for ISE
ref_traj = np.where(np.arange(STEPS) < PHASE1, REF_HI, REF_LO)
ise_no_aw   = ise(ref_traj - y_no_aw,   Ts)
ise_with_aw = ise(ref_traj - y_with_aw, Ts)

print(f"\n  ISE without anti-windup: {ise_no_aw:.4f}")
print(f"  ISE with    anti-windup: {ise_with_aw:.4f}")

results = {}
results["antiwindup_reduces_ise"] = ise_with_aw < ise_no_aw
print(f"  {'[PASS]' if results['antiwindup_reduces_ise'] else '[FAIL]'} "
      f"anti-windup reduces ISE")

# After phase 2 starts, AW controller should recover to near 0 quickly
ss_with_aw = float(np.mean(y_with_aw[-200:]))
ss_no_aw   = float(np.mean(y_no_aw[-200:]))
print(f"\n  SS with AW: {ss_with_aw:.4f}  (expect near 0)")
print(f"  SS no  AW:  {ss_no_aw:.4f}")
results["ss_reachable"] = abs(ss_with_aw) < 0.1
print(f"  {'[PASS]' if results['ss_reachable'] else '[FAIL]'} "
      f"AW controller recovers to near 0 in phase 2")

results["no_aw_finite"] = np.all(np.isfinite(y_no_aw))
print(f"  {'[PASS]' if results['no_aw_finite'] else '[FAIL]'} no-AW output is finite")

peak_no_aw   = float(np.max(y_no_aw))
peak_with_aw = float(np.max(y_with_aw))
print(f"\n  Peak output - no AW: {peak_no_aw:.4f}, with AW: {peak_with_aw:.4f}")
results["aw_peak_lower"] = peak_with_aw <= peak_no_aw + 0.1
print(f"  {'[PASS]' if results['aw_peak_lower'] else '[FAIL]'} "
      f"anti-windup does not increase peak")

print_summary(results)
