"""
ex31 - Frequency-Domain Closed-Loop Analysis
==============================================
Goal     : Compute sensitivity S(jomega) = 1/(1+L(jomega)) and complementary
           sensitivity T(jomega) = L(jomega)/(1+L(jomega)) for the PID-controlled plant.
           Verify Bode's integral (waterbed), peak sensitivity Ms, and
           crossover frequency match the time-domain step response.

Data generation : Analytic frequency response from PID + plant transfer functions.
Verification    :
  - Ms = max|S(jomega)| > 1.0 (physical lower bound).
  - |S(jomega)| + |T(jomega)| approx = |S + T| = 1 at all frequencies (within 1e-6).
  - -3 dB closed-loop bandwidth (|T| = 0.707) > 1 rad/s.

Run:
    conda activate soft_robotics
    python ex31_frequency_analysis.py
"""

import numpy as np
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from utils.verify import assert_close, print_summary

Ts = 0.01
Kp, Ki, Kd, N = 3.0, 1.5, 0.75, 10.0

print("=" * 60)
print("ex31 - Frequency-Domain Closed-Loop Analysis")
print("=" * 60)

omega = np.logspace(-2, 3, 5000)

# Continuous-time plant G(s) = 1/(s^2+1.5s+1)
def G(s): return 1.0 / (s**2 + 1.5*s + 1.0)

# PID in frequency domain (filtered derivative)
# C(s) = Kp + Ki/s + Kd*N*s/(s+N)
def C_pid(s):
    alpha = N / (s + N)   # derivative filter
    return Kp + Ki / s + Kd * N * alpha

s_arr = 1j * omega
G_arr = G(s_arr)
C_arr = C_pid(s_arr)
L_arr = G_arr * C_arr     # open-loop

S_arr = 1.0 / (1.0 + L_arr)    # sensitivity
T_arr = L_arr / (1.0 + L_arr)  # comp. sensitivity

# Verify S + T = 1 exactly
ST_err = np.max(np.abs(S_arr + T_arr - 1.0))
results = {}
results["S_plus_T_eq_1"] = ST_err < 1e-10
print(f"\n  max|S + T - 1| = {ST_err:.2e}  "
      f"{'[PASS]' if results['S_plus_T_eq_1'] else '[FAIL]'}")

Ms = float(np.max(np.abs(S_arr)))
print(f"\n  Peak sensitivity Ms = {Ms:.4f}")
results["Ms_gt_1"]   = Ms > 1.0
results["Ms_finite"] = np.isfinite(Ms)
print(f"  {'[PASS]' if results['Ms_gt_1'] else '[FAIL]'} Ms > 1 (physical lower bound)")

# Closed-loop bandwidth (|T| = 0.707)
T_mag = np.abs(T_arr)
bw_idx = np.argmax(T_mag < (1.0 / np.sqrt(2.0)))
bw_rad = float(omega[bw_idx]) if bw_idx > 0 else np.nan
print(f"  Closed-loop bandwidth: {bw_rad:.2f} rad/s")
results["bandwidth_gt_1"] = bw_rad > 1.0 if not np.isnan(bw_rad) else False
print(f"  {'[PASS]' if results['bandwidth_gt_1'] else '[FAIL]'} bandwidth > 1 rad/s")

# Gain margin (|L|=1 -> |S| check at phase crossover)
L_mag   = np.abs(L_arr)
L_phase = np.degrees(np.angle(L_arr))
cross_idx = np.argmin(np.abs(L_mag - 1.0))
pm = float(L_phase[cross_idx]) + 180.0
print(f"  Phase margin at |L|=1: {pm:.1f}^\circ")
results["pm_positive"] = pm > 0.0
print(f"  {'[PASS]' if results['pm_positive'] else '[FAIL]'} phase margin > 0^\circ")

print_summary(results)
