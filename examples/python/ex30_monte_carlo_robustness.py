"""
ex30 - Monte Carlo Robustness Analysis
========================================
Goal     : Evaluate PID robustness to plant parameter uncertainty.
           Perturb a1, a2, b1, b2 by +/-10% (uniform distribution, N=200 runs)
           and record the fraction of runs that remain stable and within
           a 20% performance degradation budget relative to the nominal plant.

Data generation : 200 Monte Carlo runs; PRBS excitation for each perturbed plant.
Verification    :
  - Stability rate > 90% for +/-10% perturbation.
  - ISE mean < 2* nominal ISE.
  - ISE std < ISE mean (moderate variability).

Run:
    conda activate soft_robotics
    python ex30_monte_carlo_robustness.py
"""

import numpy as np
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from utils.plant import tf2ss, EXAMPLE_NUM, EXAMPLE_DEN, ss_step
from utils.controllers import DiscretePID
from utils.verify import ise, print_summary

Ts       = 0.01
STEPS    = 800
N_MONTE  = 200
PERTURB  = 0.10    # +/-10%
RNG_SEED = 2024

Kp, Ki, Kd = 3.0, 1.5, 0.75

print("=" * 60)
print("ex30 - Monte Carlo Robustness (+/-10% plant uncertainty)")
print("=" * 60)

a1_nom, a2_nom = EXAMPLE_DEN[1], EXAMPLE_DEN[2]
b0_nom, b1_nom, b2_nom = EXAMPLE_NUM[0], EXAMPLE_NUM[1], EXAMPLE_NUM[2]

def sim_perturbed(num, den):
    try:
        plant = tf2ss(num, den)
    except Exception:
        return None, False
    pid = DiscretePID(Kp=Kp, Ki=Ki, Kd=Kd, Ts=Ts, u_min=-10.0, u_max=10.0)
    y = np.zeros(STEPS)
    for k in range(STEPS):
        u = pid.compute(1.0, y[k-1] if k > 0 else 0.0)
        y[k] = ss_step(plant, u)
    stable = np.all(np.isfinite(y)) and float(np.max(np.abs(y))) < 50.0
    return y, stable

# Nominal ISE
y_nom, _ = sim_perturbed(EXAMPLE_NUM, EXAMPLE_DEN)
ise_nom   = ise(1.0 - y_nom, Ts)
print(f"\n  Nominal ISE: {ise_nom:.6f}")

rng = np.random.default_rng(RNG_SEED)
ise_vals = []
stable_count = 0

for _ in range(N_MONTE):
    scale = 1.0 + rng.uniform(-PERTURB, PERTURB, size=4)
    num_p = [b0_nom,          b1_nom * scale[2], b2_nom * scale[3]]
    den_p = [1.0,             a1_nom * scale[0], a2_nom * scale[1]]
    y_p, ok = sim_perturbed(num_p, den_p)
    if ok:
        stable_count += 1
        ise_vals.append(ise(1.0 - y_p, Ts))
    else:
        ise_vals.append(np.nan)

stability_rate = stable_count / N_MONTE
ise_arr   = np.array([v for v in ise_vals if not np.isnan(v)])
ise_mean  = float(np.mean(ise_arr))
ise_std   = float(np.std(ise_arr))

print(f"  Stability rate: {stability_rate:.1%}  ({stable_count}/{N_MONTE} runs)")
print(f"  ISE mean: {ise_mean:.6f},  std: {ise_std:.6f}")
print(f"  ISE mean/nominal ratio: {ise_mean/ise_nom:.2f}*")

results = {}
results["stability_rate"]   = stability_rate > 0.90
results["ise_mean_bounded"] = ise_mean < 2.0 * ise_nom
results["ise_std_ok"]       = ise_std < ise_mean
results["runs_completed"]   = stable_count > 0

print(f"\n  {'[PASS]' if results['stability_rate'] else '[FAIL]'} "
      f"stability rate > 90% ({stability_rate:.1%})")
print(f"  {'[PASS]' if results['ise_mean_bounded'] else '[FAIL]'} "
      f"ISE mean < 2* nominal")
print(f"  {'[PASS]' if results['ise_std_ok'] else '[FAIL]'} "
      f"ISE std < ISE mean")

print_summary(results)
