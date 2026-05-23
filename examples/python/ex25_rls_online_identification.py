"""
ex25 - Online Recursive Least Squares (RLS) ARX Identification
================================================================
Goal     : Implement RLS with forgetting factor lambda=0.98 and identify a 2nd-order
           ARX model online as the plant is driven by PRBS excitation.
           Verify that identified parameters converge to true values.

Data generation : 3 000 PRBS samples; parameters updated every step.
Verification    :
  - After 1 500 samples, a1_rls error < 0.1% of true value.
  - Parameter trace is monotonically convergent (std of last-500 < std of 500-1000).

Run:
    conda activate soft_robotics
    python ex25_rls_online_identification.py
"""

import numpy as np
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from utils.plant import example_plant, ss_step, EXAMPLE_DEN, EXAMPLE_NUM
from utils.data_gen import prbs
from utils.verify import print_summary, assert_close

Ts     = 0.01
STEPS  = 3000
LAMBDA = 0.98    # forgetting factor

print("=" * 60)
print("ex25 - Online RLS ARX Identification")
print("=" * 60)
print(f"\n  lambda={LAMBDA}, na=2, nb=2, {STEPS} samples")

_, u = prbs(STEPS, Ts, amplitude=0.5, seed=31)
plant = example_plant()
y = np.zeros(STEPS)
for k in range(STEPS):
    y[k] = ss_step(plant, u[k])

# RLS state: θ = [a1, a2, b1, b2]
n_params = 4
theta = np.zeros(n_params)
P     = 1e4 * np.eye(n_params)

theta_hist = np.zeros((STEPS, n_params))

for k in range(2, STEPS):
    phi = np.array([-y[k-1], -y[k-2], u[k-1], u[k-2]])
    y_hat = float(phi @ theta)
    eps   = y[k] - y_hat

    denom = LAMBDA + phi @ P @ phi
    L     = (P @ phi) / denom
    theta = theta + L * eps
    P     = (P - np.outer(L, phi @ P)) / LAMBDA

    theta_hist[k] = theta

a1_true, a2_true = EXAMPLE_DEN[1], EXAMPLE_DEN[2]
b1_true, b2_true = EXAMPLE_NUM[1], EXAMPLE_NUM[2]

a1_final = theta_hist[-1, 0]
a2_final = theta_hist[-1, 1]
b1_final = theta_hist[-1, 2]
b2_final = theta_hist[-1, 3]

print(f"\n  Final estimates vs true values:")
print(f"    a1: {a1_final:.7f}  (true: {a1_true:.7f})")
print(f"    a2: {a2_final:.7f}  (true: {a2_true:.7f})")
print(f"    b1: {b1_final:.9f}  (true: {b1_true:.9f})")
print(f"    b2: {b2_final:.9f}  (true: {b2_true:.9f})")

results = {}
HALF = STEPS // 2
results["a1_converged"] = assert_close(a1_final, a1_true,
    tol=abs(a1_true)*0.001, label="a1 converged (0.1%)")
results["a2_converged"] = assert_close(a2_final, a2_true,
    tol=abs(a2_true)*0.001, label="a2 converged (0.1%)")
results["b1_converged"] = assert_close(b1_final, b1_true,
    tol=abs(b1_true)*0.01,  label="b1 converged (1%)")

# Convergence: std of a1 trace in last 500 < std in middle 500
std_mid  = float(np.std(theta_hist[HALF-250:HALF+250, 0]))
std_late = float(np.std(theta_hist[-500:, 0]))
results["converging"] = std_late < std_mid
print(f"\n  a1 trace std - mid: {std_mid:.2e},  late: {std_late:.2e}")
print(f"  {'[PASS]' if results['converging'] else '[FAIL]'} RLS converging (late std < mid std)")

print_summary(results)
