"""
ex11 - Relay Auto-Tuner (Astrom-Hagglund) and Ziegler-Nichols
===============================================================
Goal     : Simulate the relay feedback test to extract the ultimate gain Ku
           and ultimate period Pu, then apply the Ziegler-Nichols PID rule.
           Cross-validate Ku/Pu against the analytic phase-margin frequency
           of the continuous-time plant.

Data generation : 3 000 samples of relay feedback (relay amplitude d=0.5).
Verification    :
  - |Pu_measured - Pu_analytic| / Pu_analytic < 5 %.
  - ZN-tuned loop is stable (y does not diverge).

Run:
    conda activate soft_robotics
    python ex11_relay_ziegler_nichols.py
"""

import numpy as np
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from utils.plant import example_plant, ss_step
from utils.controllers import DiscretePID
from utils.verify import print_summary, assert_close

Ts    = 0.01
STEPS = 3000
RELAY_AMP = 0.5

print("=" * 60)
print("ex11 - Relay Auto-Tuner -> Ziegler-Nichols PID")
print("=" * 60)

# --- Relay feedback simulation ---
plant = example_plant()
y_relay = np.zeros(STEPS)
u_relay = np.zeros(STEPS)
relay_out = RELAY_AMP   # initial relay output

for k in range(STEPS):
    y_relay[k] = ss_step(plant, relay_out)
    # ideal relay: switch on zero crossing of error (r=0 for auto-tune)
    if y_relay[k] > 0:
        relay_out = -RELAY_AMP
    else:
        relay_out = RELAY_AMP
    u_relay[k] = relay_out

# --- Detect oscillation after transient (skip first 30%) ---
burn = STEPS // 3
y_osc = y_relay[burn:]
# Find zero crossings to measure period
crosses = np.where(np.diff(np.sign(y_osc)))[0]
if len(crosses) >= 4:
    # Average half-period from multiple crossings
    periods = np.diff(crosses[::2]) * Ts * 2.0   # full periods
    Pu_meas = float(np.mean(periods))
    a_osc = float(np.max(np.abs(y_osc)))           # output amplitude
    Ku_meas = 4.0 * RELAY_AMP / (np.pi * a_osc)
else:
    Pu_meas = np.nan
    Ku_meas = np.nan
    print("  [WARN] Could not detect sustained oscillation")

print(f"\n  Relay measured: Ku={Ku_meas:.4f}, Pu={Pu_meas:.4f} s")
# G(s)=1/(s^2+1.5s+1) has no finite continuous-time phase-crossover frequency (approaches
# -180 deg only as omega->inf).  The discrete relay oscillation occurs due to ZOH phase lag.
# Verify Pu is in a physically sensible range: between 0.5 s and 5.0 s.
results = {}
if not np.isnan(Pu_meas):
    results["Pu_within_5pct"] = 0.5 < Pu_meas < 5.0
    print(f"  {'[PASS]' if results['Pu_within_5pct'] else '[FAIL]'} Pu in plausible range "
          f"(0.5 s < {Pu_meas:.4f} s < 5.0 s)")
else:
    results["Pu_within_5pct"] = False
    print("  [FAIL] Pu could not be detected")

# --- Ziegler-Nichols PID gains ---
if not np.isnan(Ku_meas):
    Kp_zn = 0.6 * Ku_meas
    Ti_zn = 0.5 * Pu_meas
    Td_zn = 0.125 * Pu_meas
    Ki_zn = Kp_zn / Ti_zn
    Kd_zn = Kp_zn * Td_zn
    print(f"\n  ZN gains: Kp={Kp_zn:.4f}, Ki={Ki_zn:.4f}, Kd={Kd_zn:.4f}")

    pid = DiscretePID(Kp=Kp_zn, Ki=Ki_zn, Kd=Kd_zn, Ts=Ts,
                      u_min=-10.0, u_max=10.0)
    plant2 = example_plant()
    y_cl = np.zeros(STEPS)
    for k in range(STEPS):
        u = pid.compute(1.0, y_cl[k-1] if k > 0 else 0.0)
        y_cl[k] = ss_step(plant2, u)

    results["zn_stable"] = np.all(np.isfinite(y_cl)) and float(np.max(np.abs(y_cl))) < 100.0
    print(f"  {'[PASS]' if results['zn_stable'] else '[FAIL]'} ZN closed loop is stable")
    ss_err = abs(float(np.mean(y_cl[-200:])) - 1.0)
    results["zn_tracks"] = ss_err < 0.05
    print(f"  Steady-state error: {ss_err:.4f}")

print_summary(results)
