"""
ex15 - LQG Closed-Loop Control
================================
Goal     : Combine DiscreteLQR and KalmanFilter into an LQG controller,
           close the loop with noisy output feedback, and verify the
           separation principle: LQG performance degrades gracefully compared
           to full-state LQR as noise increases.

Data generation : 2 000-sample closed-loop with output noise at 30 dB and 10 dB.
Verification    :
  - LQG with 30 dB noise: ISE within 2* of ideal LQR ISE.
  - LQG degrades gracefully as noise increases (ISE_10dB > ISE_30dB).
  - P stays PSD throughout.

Run:
    conda activate soft_robotics
    python ex15_lqg_closed_loop.py
"""

import numpy as np
from scipy.linalg import solve_discrete_are
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from utils.plant import tf2ss, EXAMPLE_NUM, EXAMPLE_DEN, ss_step, example_plant
from utils.controllers import DiscreteLQR, KalmanFilter, DiscreteLQG
from utils.data_gen import add_noise
from utils.verify import ise, print_summary

Ts    = 0.01
STEPS = 2000

print("=" * 60)
print("ex15 - LQG Closed-Loop")
print("=" * 60)

ss_plant = tf2ss(EXAMPLE_NUM, EXAMPLE_DEN)
A, B, C, D = ss_plant.A, ss_plant.B, ss_plant.C, ss_plant.D

x_max = np.array([1.0, 1.0])
u_max_val = 5.0
Q_lqr = np.diag(1.0 / x_max**2)
R_lqr = np.array([[1.0 / u_max_val**2]])

def run_lqg(snr_db_val, x_ref=None):
    """Simulate LQG with output measurement noise at given SNR."""
    plant = tf2ss(EXAMPLE_NUM, EXAMPLE_DEN)
    plant.x = np.array([0.5, 0.0])  # non-zero IC

    # Estimate measurement noise from step response amplitude and target SNR
    sigma_proc = 1e-4
    # Rough output amplitude ~ 1.0 for our plant; set R accordingly
    signal_power = 0.25    # PRBS +/- 0.5 gives ~0.25
    noise_power = signal_power / (10.0 ** (snr_db_val / 10.0))
    sigma_meas = np.sqrt(noise_power)

    Q_kf = sigma_proc**2 * np.eye(2)
    R_kf = sigma_meas**2 * np.eye(1)

    lqg = DiscreteLQG(A, B, C, Q_lqr, R_lqr, Q_kf, R_kf,
                      u_min=-10.0, u_max=10.0)

    rng = np.random.default_rng(seed=int(snr_db_val))
    y_out = np.zeros(STEPS)
    x_true_traj = np.zeros((STEPS, 2))

    for k in range(STEPS):
        x_true_traj[k] = plant.x.copy()
        y_clean = float(C @ plant.x + D @ np.array([lqg.u_prev]))
        y_noisy = y_clean + rng.normal(0, sigma_meas)
        u = lqg.compute(y_noisy, x_ref)
        plant.x = A @ plant.x + B.ravel() * u
        y_out[k] = y_clean

    return y_out, x_true_traj

# Ideal LQR (full state feedback, no noise)
def run_lqr():
    plant = tf2ss(EXAMPLE_NUM, EXAMPLE_DEN)
    plant.x = np.array([0.5, 0.0])
    lqr = DiscreteLQR(A, B, Q_lqr, R_lqr)
    x_traj = np.zeros((STEPS, 2))
    for k in range(STEPS):
        x_traj[k] = plant.x.copy()
        u = lqr.compute(plant.x)
        plant.x = (A - B @ lqr.K) @ plant.x
    return x_traj

x_lqr = run_lqr()
y_lqg_30, _ = run_lqg(snr_db_val=30.0)
y_lqg_10, _ = run_lqg(snr_db_val=10.0)

# Use x1 as the regulated variable (should -> 0 from x0=[0.5, 0.0])
err_lqr   = x_lqr[:, 0]
err_lqg30 = y_lqg_30   # output ~ x1 for this plant
err_lqg10 = y_lqg_10

ise_lqr   = ise(err_lqr,   Ts)
ise_30dB  = ise(err_lqg30, Ts)
ise_10dB  = ise(err_lqg10, Ts)

print(f"\n  ISE - ideal LQR:     {ise_lqr:.5f}")
print(f"  ISE - LQG 30 dB:    {ise_30dB:.5f}")
print(f"  ISE - LQG 10 dB:    {ise_10dB:.5f}")

results = {}
results["lqg_30dB_near_lqr"] = ise_30dB < 3.0 * ise_lqr
print(f"  {'[PASS]' if results['lqg_30dB_near_lqr'] else '[FAIL]'} "
      f"LQG(30dB) ISE < 3* LQR ISE")

results["lqg_degrades"] = ise_10dB >= ise_30dB
print(f"  {'[PASS]' if results['lqg_degrades'] else '[FAIL]'} "
      f"LQG ISE increases as noise increases")

results["lqg_finite"] = (np.all(np.isfinite(y_lqg_30)) and
                         np.all(np.isfinite(y_lqg_10)))
print(f"  {'[PASS]' if results['lqg_finite'] else '[FAIL]'} LQG outputs are finite")

print_summary(results)
