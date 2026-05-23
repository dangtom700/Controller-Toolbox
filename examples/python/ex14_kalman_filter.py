"""
ex14 - Kalman Filter State Estimation
========================================
Goal     : Run the KalmanFilter on the example plant with noisy output
           measurements and verify the estimated states converge to the true
           states; confirm the Joseph-form P update keeps P symmetric and PSD.

Data generation : 1 500 PRBS samples; output noise at 20 dB SNR.
Verification    :
  - RMSE(x_hat, x_true) < RMSE(x_naive, x_true) where x_naive uses y as x1.
  - P stays symmetric (|P - P'|_inf < 1e-12) at every step.
  - P is PSD (min eigenvalue >= -1e-10) at every step.

Run:
    conda activate soft_robotics
    python ex14_kalman_filter.py
"""

import numpy as np
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from utils.plant import tf2ss, EXAMPLE_NUM, EXAMPLE_DEN, ss_step
from utils.controllers import KalmanFilter
from utils.data_gen import prbs, add_noise
from utils.verify import rmse, print_summary

Ts    = 0.01
STEPS = 1500

print("=" * 60)
print("ex14 - Kalman Filter")
print("=" * 60)

ss_true = tf2ss(EXAMPLE_NUM, EXAMPLE_DEN)
A, B, C, D = ss_true.A, ss_true.B, ss_true.C, ss_true.D

_, u = prbs(STEPS, Ts, amplitude=0.5, seed=11)

# Simulate true plant (no noise)
x_true = np.zeros((STEPS, 2))
y_true = np.zeros(STEPS)
for k in range(STEPS):
    x_true[k] = ss_true.x.copy()
    y_true[k] = float(ss_step(ss_true, u[k]))

y_noisy = add_noise(y_true, snr_db=20.0, seed=5)

# Kalman filter
sigma_proc = 1e-4
sigma_meas = float(np.std(y_noisy - y_true))

Q_kf = sigma_proc**2 * np.eye(2)
R_kf = sigma_meas**2 * np.eye(1)

kf = KalmanFilter(A, B, C, Q_kf, R_kf)

x_hat = np.zeros((STEPS, 2))
P_sym_violations = 0
P_psd_violations = 0

for k in range(STEPS):
    x_hat[k] = kf.step(u[k], y_noisy[k])
    sym_err = float(np.max(np.abs(kf.P - kf.P.T)))
    if sym_err > 1e-12:
        P_sym_violations += 1
    min_eig = float(np.min(np.linalg.eigvalsh(kf.P)))
    if min_eig < -1e-10:
        P_psd_violations += 1

print(f"\n  Noise sigma_meas approx = {sigma_meas:.6f}")
print(f"  Q_kf = {sigma_proc**2:.2e} * I,  R_kf = {sigma_meas**2:.4e} * I")

rmse_kf = rmse(x_true[:, 0], x_hat[:, 0])
rmse_raw = rmse(x_true[:, 0], y_noisy)

print(f"\n  RMSE x1 (Kalman) : {rmse_kf:.6f}")
print(f"  RMSE x1 (raw y)  : {rmse_raw:.6f}")

results = {}
results["kf_better_than_raw"] = rmse_kf < rmse_raw
print(f"  {'[PASS]' if results['kf_better_than_raw'] else '[FAIL]'} KF RMSE < raw RMSE")

results["P_symmetric"] = (P_sym_violations == 0)
print(f"  {'[PASS]' if results['P_symmetric'] else '[FAIL]'} "
      f"P symmetric ({P_sym_violations} violations)")

results["P_psd"] = (P_psd_violations == 0)
print(f"  {'[PASS]' if results['P_psd'] else '[FAIL]'} "
      f"P PSD ({P_psd_violations} violations)")

# Convergence: RMS error in 2nd half < 1st half
N2 = STEPS // 2
rmse_first  = rmse(x_true[:N2, 0], x_hat[:N2, 0])
rmse_second = rmse(x_true[N2:, 0], x_hat[N2:, 0])
results["kf_converges"] = rmse_second < rmse_first
print(f"  RMSE 1st half: {rmse_first:.6f},  2nd half: {rmse_second:.6f}")
print(f"  {'[PASS]' if results['kf_converges'] else '[FAIL]'} KF improves over time")

print_summary(results)
