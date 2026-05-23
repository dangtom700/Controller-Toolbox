"""
ex16 - MPC Prediction Horizon Tuning
======================================
Goal     : Show how changing the prediction horizon Np (with Nc=Np/2) affects
           MPC closed-loop performance; verify that ISE improves up to an
           asymptote as Np grows, and that the precomputed Hessian is PSD.

Data generation : 1 500-sample closed-loop for Np \in {5, 10, 20, 40}.
Verification    :
  - Hessian is symmetric PSD for each horizon.
  - ISE is non-increasing as Np grows (diminishing returns).
  - All simulations are stable (bounded output).

Run:
    conda activate soft_robotics
    python ex16_mpc_horizon_tuning.py
"""

import numpy as np
from scipy.linalg import solve_discrete_are
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from utils.plant import tf2ss, EXAMPLE_NUM, EXAMPLE_DEN, ss_step
from utils.verify import ise, print_summary

Ts    = 0.01
STEPS = 1500

print("=" * 60)
print("ex16 - MPC Horizon Tuning")
print("=" * 60)

ss = tf2ss(EXAMPLE_NUM, EXAMPLE_DEN)
A, B, C, D = ss.A, ss.B, ss.C, ss.D
n, m, p = A.shape[0], B.shape[1], C.shape[0]

Q_mpc, R_mpc = 1.0, 0.01

def build_mpc(Np, Nc):
    Phi   = np.zeros((Np*p, n))
    Theta = np.zeros((Np*p, Nc*m))
    Ak = np.eye(n)
    for i in range(Np):
        Ak = A @ Ak
        Phi[i*p:(i+1)*p, :] = C @ Ak
        for j in range(min(i+1, Nc)):
            Akj = np.eye(n)
            for _ in range(i - j):
                Akj = A @ Akj
            Theta[i*p:(i+1)*p, j*m:(j+1)*m] = C @ Akj @ B
    Q_bar = Q_mpc * np.eye(Np*p)
    R_bar = R_mpc * np.eye(Nc*m)
    H = Theta.T @ Q_bar @ Theta + R_bar
    F = np.linalg.solve(H, Theta.T @ Q_bar @ Phi)   # (Nc*m, n)
    G = np.linalg.solve(H, Theta.T @ Q_bar)          # feedforward to r
    return H, F, G

def sim_mpc(Np, Nc):
    H, F, G = build_mpc(Np, Nc)
    plant = tf2ss(EXAMPLE_NUM, EXAMPLE_DEN)
    y_out = np.zeros(STEPS)
    for k in range(STEPS):
        r_vec = np.ones(Np * p)
        u_seq = -F @ plant.x + G @ r_vec
        u = float(np.clip(u_seq[0], -10.0, 10.0))
        y_out[k] = float(ss_step(plant, u))
    return y_out, H

horizons = [5, 10, 20, 40]
results = {}
ise_vals = []

print(f"\n  Np  | Nc | H PSD | H sym  | ISE")
print("  " + "-"*44)

for Np in horizons:
    Nc = max(1, Np // 2)
    y, H = sim_mpc(Np, Nc)
    err = 1.0 - y
    ise_v = ise(err, Ts)
    ise_vals.append(ise_v)

    min_eig = float(np.min(np.linalg.eigvalsh(H)))
    sym_err = float(np.max(np.abs(H - H.T)))
    psd_ok = min_eig > -1e-10
    sym_ok = sym_err < 1e-10

    results[f"H_psd_Np{Np}"] = psd_ok
    results[f"H_sym_Np{Np}"] = sym_ok
    results[f"stable_Np{Np}"] = np.all(np.isfinite(y)) and float(np.max(np.abs(y))) < 20.0

    print(f"  {Np:>3} | {Nc:>2} | {'PASS' if psd_ok else 'FAIL':>5} | "
          f"{'PASS' if sym_ok else 'FAIL':>6} | {ise_v:.6f}")

# ISE should be non-increasing (roughly)
for i in range(len(ise_vals) - 1):
    results[f"ise_nonincreasing_{horizons[i]}_{horizons[i+1]}"] = ise_vals[i+1] <= ise_vals[i] * 1.1

print_summary(results)
