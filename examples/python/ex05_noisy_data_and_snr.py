"""
ex05 - Noisy Measurement Data and SNR Verification
===================================================
Audience : Experienced developers familiar with the C++ controller_toolbox.
Goal     : Add white Gaussian noise at a specified SNR to a plant step
           response, verify the achieved SNR, and demonstrate how noise
           degrades ARX identification compared to the clean case (ex03).

Data generation : 1 500 step samples + noise at 20 dB, 30 dB, 40 dB SNR.
Verification    :
  - Achieved SNR within +/-1 dB of target for each case.
  - ARX coefficient error grows monotonically as SNR decreases.

Run:
    conda activate soft_robotics
    python ex05_noisy_data_and_snr.py
"""

import numpy as np
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from utils.plant import example_plant, ss_step, EXAMPLE_DEN, EXAMPLE_NUM
from utils.data_gen import prbs, add_noise
from utils.verify import snr_db, assert_close, print_summary

Ts    = 0.01
STEPS = 1500

print("=" * 60)
print("ex05 - Noisy Data and SNR Verification")
print("=" * 60)

# --- Generate clean I/O data (PRBS) ---
_, u = prbs(STEPS, Ts, amplitude=0.5, seed=42)
plant = example_plant()
y_clean = np.zeros(STEPS)
for k in range(STEPS):
    y_clean[k] = ss_step(plant, u[k])

print(f"\nClean I/O: {STEPS} PRBS samples")

# --- ARX identification helper ---
def arx2_identify(u, y, burn=50):
    N = len(y) - burn - 2
    Phi = np.column_stack([
        -y[burn+1:burn+1+N], -y[burn:burn+N],
         u[burn+1:burn+1+N],  u[burn:burn+N],
    ])
    Y_vec = y[burn+2:burn+2+N]
    theta, *_ = np.linalg.lstsq(Phi, Y_vec, rcond=None)
    return theta   # [a1, a2, b1, b2]

theta_clean = arx2_identify(u, y_clean)
a1_true, a2_true = EXAMPLE_DEN[1], EXAMPLE_DEN[2]
b1_true, b2_true = EXAMPLE_NUM[1], EXAMPLE_NUM[2]

print("\n  SNR (dB) | achieved SNR | a1 error (%) | b1 error (%)")
print("  " + "-"*62)

results = {}
snr_targets = [40.0, 30.0, 20.0]
a1_errors = []

for target_snr in snr_targets:
    y_noisy = add_noise(y_clean, snr_db=target_snr, seed=int(target_snr))
    achieved = snr_db(y_clean, y_noisy)
    theta = arx2_identify(u, y_noisy)
    a1_err_pct = 100.0 * abs(theta[0] - a1_true) / abs(a1_true)
    b1_err_pct = 100.0 * abs(theta[2] - b1_true) / abs(b1_true)
    a1_errors.append(a1_err_pct)

    snr_ok = abs(achieved - target_snr) < 1.0
    results[f"SNR {target_snr:.0f}dB achieved"] = snr_ok
    tag = "[PASS]" if snr_ok else "[FAIL]"
    print(f"  {target_snr:>9.0f} | {achieved:>12.2f} | {a1_err_pct:>12.4f} | {b1_err_pct:>12.4f}  {tag}")

# --- Check clean identification is very accurate ---
theta_c = arx2_identify(u, y_clean)
results["a1_clean_1pct"] = assert_close(theta_c[0], a1_true,
                                         tol=abs(a1_true)*0.01, label="a1 clean (1%)")
results["b1_clean_1pct"] = assert_close(theta_c[2], b1_true,
                                         tol=abs(b1_true)*0.01, label="b1 clean (1%)")

# --- Check coefficient error increases as SNR decreases ---
monotone_ok = (a1_errors[0] < a1_errors[1] < a1_errors[2])
results["err_increases_with_noise"] = monotone_ok
print(f"\n  a1 errors by SNR (40->30->20 dB): {[f'{e:.4f}%' for e in a1_errors]}")
print(f"  {'[PASS]' if monotone_ok else '[FAIL]'} error monotonically increases as SNR decreases")

print_summary(results)
