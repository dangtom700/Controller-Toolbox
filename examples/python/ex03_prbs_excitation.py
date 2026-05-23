"""
ex03 - PRBS Excitation and ARX Model Identification
====================================================
Audience : Experienced developers familiar with the C++ controller_toolbox.
Goal     : Generate a PRBS input, drive the example plant in open loop, then
           fit a 2nd-order ARX model using least squares and compare the
           identified coefficients against the known plant coefficients.

Data generation : 2 000 samples of PRBS(+/-0.5) through ss_step().
Verification    :
  - PRBS has near-flat power spectrum (ratio max/min spectral power < 10 dB).
  - ARX identified coefficients match plant coefficients to < 1 %.

Run:
    conda activate soft_robotics
    python ex03_prbs_excitation.py
"""

import numpy as np
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from utils.plant import example_plant, ss_step, EXAMPLE_DEN, EXAMPLE_NUM
from utils.data_gen import prbs
from utils.verify import assert_close, print_summary

Ts    = 0.01
STEPS = 2000

print("=" * 60)
print("ex03 - PRBS Excitation & ARX Identification")
print("=" * 60)

# --- Generate PRBS and collect I/O data ---
_, u = prbs(STEPS, Ts, amplitude=0.5, seed=7)
plant = example_plant()
y = np.zeros(STEPS)
for k in range(STEPS):
    y[k] = ss_step(plant, u[k])

print(f"\nGenerated {STEPS} PRBS samples, amplitude=+/-0.5")

# --- Verify PRBS spectral flatness ---
Y = np.abs(np.fft.rfft(u))**2
Y_db = 10.0 * np.log10(Y[1:] + 1e-30)   # skip DC
spread_db = float(np.max(Y_db) - np.min(Y_db))
print(f"\n  PRBS spectral spread (max-min): {spread_db:.1f} dB")
results = {}
results["prbs_flat"] = (spread_db < 40.0)
print(f"  {'[PASS]' if results['prbs_flat'] else '[FAIL]'} spectral spread < 40 dB")

# --- ARX identification: 2nd order, na=2, nb=2 ---
# y[k] = -a1 y[k-1] - a2 y[k-2] + b1 u[k-1] + b2 u[k-2]
# Build regression matrix [-y[k-1], -y[k-2], u[k-1], u[k-2]]
BURN = 50   # discard transient
N  = STEPS - BURN - 2
Phi = np.column_stack([
    -y[BURN+1:BURN+1+N],
    -y[BURN:BURN+N],
     u[BURN+1:BURN+1+N],
     u[BURN:BURN+N],
])
Y_vec = y[BURN+2:BURN+2+N]
theta, *_ = np.linalg.lstsq(Phi, Y_vec, rcond=None)
a1_id, a2_id, b1_id, b2_id = theta

print(f"\n  ARX identified coefficients:")
print(f"    a1 = {a1_id:.6f}  (true: {EXAMPLE_DEN[1]:.6f})")
print(f"    a2 = {a2_id:.6f}  (true: {EXAMPLE_DEN[2]:.6f})")
print(f"    b1 = {b1_id:.8f}  (true: {EXAMPLE_NUM[1]:.8f})")
print(f"    b2 = {b2_id:.8f}  (true: {EXAMPLE_NUM[2]:.8f})")

tol_rel = 0.01   # 1 %
results["a1"] = assert_close(a1_id, EXAMPLE_DEN[1],
                              tol=abs(EXAMPLE_DEN[1]) * tol_rel, label="a1 (1%)")
results["a2"] = assert_close(a2_id, EXAMPLE_DEN[2],
                              tol=abs(EXAMPLE_DEN[2]) * tol_rel, label="a2 (1%)")
results["b1"] = assert_close(b1_id, EXAMPLE_NUM[1],
                              tol=abs(EXAMPLE_NUM[1]) * tol_rel, label="b1 (1%)")
results["b2"] = assert_close(b2_id, EXAMPLE_NUM[2],
                              tol=abs(EXAMPLE_NUM[2]) * tol_rel, label="b2 (1%)")

# --- One-step-ahead prediction residual ---
y_hat = Phi @ theta
residual_rms = float(np.sqrt(np.mean((Y_vec - y_hat)**2)))
print(f"\n  One-step-ahead residual RMS: {residual_rms:.3e}")
results["residual_small"] = (residual_rms < 1e-6)
print(f"  {'[PASS]' if results['residual_small'] else '[FAIL]'} residual < 1e-6")

print_summary(results)
