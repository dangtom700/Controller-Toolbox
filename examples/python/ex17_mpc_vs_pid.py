"""
ex17 - MPC vs PID Performance Comparison
==========================================
Goal     : Head-to-head comparison of MPC (Np=20) and IMC-tuned PID on the
           example plant under step + disturbance scenarios. Both controllers
           use the same u_max = 5.0 saturation limit.

Data generation : 2 000 samples; step at k=0, load disturbance +0.2 at k=1000.
Verification    :
  - MPC and PID both reject the disturbance (|y_ss - 1.0| < 1%).
  - ISE comparison is logged (no pass/fail - purely informational).
  - Both outputs bounded: |y| < 5.

Run:
    conda activate soft_robotics
    python ex17_mpc_vs_pid.py
"""

import numpy as np
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from utils.plant import tf2ss, EXAMPLE_NUM, EXAMPLE_DEN, ss_step
from utils.controllers import DiscretePID
from utils.verify import ise, itae, dc_gain_check, print_summary

Ts    = 0.01
STEPS = 2000
DIST_STEP = 1000
DIST_MAG  = 0.2
U_MAX = 5.0

print("=" * 60)
print("ex17 - MPC vs PID Comparison")
print("=" * 60)

ss_ref = tf2ss(EXAMPLE_NUM, EXAMPLE_DEN)
A, B, C, D = ss_ref.A, ss_ref.B, ss_ref.C, ss_ref.D

# --- Build MPC matrices (Np=20, Nc=10) ---
Np, Nc = 20, 10
Q_mpc, R_mpc = 1.0, 0.01
n, m, p = A.shape[0], B.shape[1], C.shape[0]

Phi   = np.zeros((Np*p, n))
Theta = np.zeros((Np*p, Nc*m))
Ak = np.eye(n)
for i in range(Np):
    Ak = A @ Ak
    Phi[i*p:(i+1)*p, :] = C @ Ak
    for j in range(min(i+1, Nc)):
        Akj = np.eye(n)
        for _ in range(i-j):
            Akj = A @ Akj
        Theta[i*p:(i+1)*p, j*m:(j+1)*m] = C @ Akj @ B

Q_bar = Q_mpc * np.eye(Np)
R_bar = R_mpc * np.eye(Nc)
H = Theta.T @ Q_bar @ Theta + R_bar
F = np.linalg.solve(H, Theta.T @ Q_bar @ Phi)
G = np.linalg.solve(H, Theta.T @ Q_bar)

# --- IMC-PID gains (lambda=0.5, FOPDT approx) ---
K_fopdt = 1.0; tau_f = 0.94; theta_f = 0.08; lam = 0.5
Kp = (tau_f + theta_f/2) / (K_fopdt * (lam + theta_f/2))
Ki = Kp / (tau_f + theta_f/2)
Kd = Kp * tau_f * theta_f / (2*tau_f + theta_f)

def sim_mpc_disturbance():
    plant = tf2ss(EXAMPLE_NUM, EXAMPLE_DEN)
    y = np.zeros(STEPS)
    for k in range(STEPS):
        r_vec = np.ones(Np)
        u_seq = -F @ plant.x + G @ r_vec
        u = float(np.clip(u_seq[0], -U_MAX, U_MAX))
        dist = DIST_MAG if k >= DIST_STEP else 0.0
        y[k] = float(ss_step(plant, u + dist))
    return y

def sim_pid_disturbance():
    pid = DiscretePID(Kp=Kp, Ki=Ki, Kd=Kd, Ts=Ts, u_min=-U_MAX, u_max=U_MAX)
    plant = tf2ss(EXAMPLE_NUM, EXAMPLE_DEN)
    y = np.zeros(STEPS)
    for k in range(STEPS):
        u = pid.compute(1.0, y[k-1] if k > 0 else 0.0)
        dist = DIST_MAG if k >= DIST_STEP else 0.0
        y[k] = float(ss_step(plant, u + dist))
    return y

y_mpc = sim_mpc_disturbance()
y_pid = sim_pid_disturbance()

err_mpc = 1.0 - y_mpc
err_pid = 1.0 - y_pid

ise_mpc = ise(err_mpc, Ts); itae_mpc = itae(err_mpc, Ts)
ise_pid = ise(err_pid, Ts); itae_pid = itae(err_pid, Ts)

print(f"\n  Metric  | MPC (Np=20)  | PID (IMC)")
print(f"  ISE     | {ise_mpc:>12.5f} | {ise_pid:>10.5f}")
print(f"  ITAE    | {itae_mpc:>12.5f} | {itae_pid:>10.5f}")

results = {}
results["mpc_bounded"] = np.all(np.isfinite(y_mpc)) and float(np.max(np.abs(y_mpc))) < 5.0
results["pid_bounded"] = np.all(np.isfinite(y_pid)) and float(np.max(np.abs(y_pid))) < 5.0

for name, y in [("MPC", y_mpc), ("PID", y_pid)]:
    ss_err = abs(float(np.mean(y[-200:])) - 1.0)
    ok = ss_err < 0.01
    results[f"{name}_ss_error"] = ok
    print(f"  {name} steady-state error: {ss_err:.4f}  "
          f"{'[PASS]' if ok else '[FAIL]'}")

print_summary(results)
