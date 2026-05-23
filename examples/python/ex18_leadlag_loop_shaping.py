"""
ex18 - Lead-Lag Controller via Loop Shaping
=============================================
Goal     : Design a lead compensator using the loop-shaping tuner (beta=sin(φ),
           alpha=(1+beta)/(1-beta)), close the loop, and verify the crossover frequency
           and phase margin match the design spec.

Data generation : Bode data from open-loop frequency response; 2 000-sample sim.
Verification    :
  - |omega_crossover_measured - omega_c_design| / omega_c_design < 20%.
  - Phase margin > 30^\circ.
  - Closed loop stable.

Run:
    conda activate soft_robotics
    python ex18_leadlag_loop_shaping.py
"""

import numpy as np
from scipy import signal as sig
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from utils.plant import example_plant, ss_step
from utils.controllers import LeadLag
from utils.verify import print_summary

Ts    = 0.01
STEPS = 2000

# Loop-shaping design parameters
omega_c = 2.0           # desired crossover frequency (rad/s)
phi_deg = 50.0          # desired phase margin (deg)

print("=" * 60)
print("ex18 - Lead-Lag Loop Shaping")
print("=" * 60)
print(f"\n  Design: omega_c = {omega_c} rad/s, φ_m = {phi_deg}^\circ")

# Compute lead compensator zero/pole (C++ LoopShapingTuner formula)
phi_rad = np.radians(phi_deg)
beta  = np.sin(phi_rad)
alpha = (1.0 + beta) / (1.0 - beta)
z_c   = omega_c / np.sqrt(alpha)
p_c   = omega_c * np.sqrt(alpha)

# Compute compensator gain so |G(jomegac).C(jomegac)| = 1
G_jw = 1.0 / ((1j*omega_c)**2 + 1.5*(1j*omega_c) + 1.0)
C_jw_unnorm = (1j*omega_c + z_c) / (1j*omega_c + p_c)
gain = 1.0 / abs(G_jw * C_jw_unnorm)

print(f"  beta={beta:.4f}, alpha={alpha:.4f}, z_c={z_c:.4f}, p_c={p_c:.4f}, gain={gain:.4f}")

ll = LeadLag(gain=gain, zero=z_c, pole=p_c, Ts=Ts, u_min=-10.0, u_max=10.0)
plant = example_plant()

y = np.zeros(STEPS)
e = np.zeros(STEPS)
for k in range(STEPS):
    e[k] = 1.0 - (y[k-1] if k > 0 else 0.0)
    u = ll.compute(e[k])
    y[k] = ss_step(plant, u)

results = {}
results["stable"] = np.all(np.isfinite(y)) and float(np.max(np.abs(y))) < 20.0
print(f"\n  {'[PASS]' if results['stable'] else '[FAIL]'} closed loop stable")

ss_err = abs(float(np.mean(y[-200:])) - 1.0)
results["ss_error"] = ss_err < 0.05
print(f"  Steady-state error: {ss_err:.4f}  "
      f"{'[PASS]' if results['ss_error'] else '[FAIL]'} < 5%")

# Open-loop Bode for phase margin check
freqs = np.logspace(-2, 2, 2000)
w_rad = freqs
G_ow = np.array([1.0/((1j*w)**2 + 1.5*(1j*w) + 1.0) for w in w_rad])
C_ow = np.array([gain * (1j*w + z_c) / (1j*w + p_c) for w in w_rad])
L_ow = G_ow * C_ow

mag = np.abs(L_ow)
phase = np.degrees(np.angle(L_ow))

# Crossover: |L(jomega)| = 1
cross_idx = np.argmin(np.abs(mag - 1.0))
wc_meas = w_rad[cross_idx]
pm_meas = phase[cross_idx] + 180.0

print(f"\n  Measured crossover: omega_c = {wc_meas:.3f} rad/s  (design: {omega_c})")
print(f"  Phase margin: {pm_meas:.1f}^\circ  (design: >{phi_deg}^\circ)")

results["crossover_20pct"] = abs(wc_meas - omega_c) / omega_c < 0.20
results["phase_margin"]    = pm_meas > 30.0

print_summary(results)
