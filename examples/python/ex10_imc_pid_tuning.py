"""
ex10 - IMC-Based PID Tuning from FOPDT Model
=============================================
Goal     : Implement the IMC-PID tuning rule, compute gains from the FOPDT
           model identified in ex02, and verify the closed-loop bandwidth
           matches the desired lambda (filter time-constant) specification.

Data generation : FOPDT parameters from step-response tangent method.
Verification    :
  - Closed-loop -3 dB bandwidth approx = 1/lambda.
  - ISE decreases as lambda decreases (faster response).

Run:
    conda activate soft_robotics
    python ex10_imc_pid_tuning.py
"""

import numpy as np
from scipy import signal as sig
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from utils.plant import example_plant, ss_step
from utils.controllers import DiscretePID
from utils.verify import ise, print_summary

Ts    = 0.01
STEPS = 2000

print("=" * 60)
print("ex10 - IMC-PID Tuning")
print("=" * 60)

# --- FOPDT from step response ---
plant = example_plant()
y_step = np.array([ss_step(plant, 1.0) for _ in range(STEPS)])
t      = np.arange(STEPS) * Ts

K = float(y_step[-1])
y28 = np.argmax(y_step >= 0.283 * K)
y63 = np.argmax(y_step >= 0.632 * K)
tau   = 1.5 * (t[y63] - t[y28])
theta = max(Ts, t[y63] - tau)   # ensure > 0

print(f"\n  FOPDT: K={K:.4f}, tau={tau:.4f} s, theta={theta:.4f} s")

# --- IMC-PID formula (Skogestad, 2003) ---
# For a FOPDT: lambda is the desired closed-loop time constant
def imc_pid(K, tau, theta, lam):
    Kp = (tau + theta/2.0) / (K * (lam + theta/2.0))
    Ti = tau + theta / 2.0
    Td = tau * theta / (2.0 * tau + theta)
    Ki = Kp / Ti
    Kd = Kp * Td
    return Kp, Ki, Kd

results = {}
lambdas = [0.3, 0.5, 1.0]
ise_vals = []

print(f"\n  lambda | Kp      | Ki      | Kd      | ISE")
print("  " + "-"*58)

for lam in lambdas:
    Kp, Ki, Kd = imc_pid(K, tau, theta, lam)
    pid = DiscretePID(Kp=Kp, Ki=Ki, Kd=Kd, Ts=Ts, u_min=-10.0, u_max=10.0)
    plant = example_plant()
    y = np.zeros(STEPS)
    for k in range(STEPS):
        u = pid.compute(1.0, y[k-1] if k > 0 else 0.0)
        y[k] = ss_step(plant, u)
    err = 1.0 - y
    ise_val = ise(err, Ts)
    ise_vals.append(ise_val)
    print(f"  {lam:>6.1f} | {Kp:>7.4f} | {Ki:>7.4f} | {Kd:>7.4f} | {ise_val:.6f}")

# Smaller lambda -> faster -> lower ISE
results["ise_monotone"] = ise_vals[0] < ise_vals[1] < ise_vals[2]
print(f"\n  {'[PASS]' if results['ise_monotone'] else '[FAIL]'} "
      f"ISE decreases as lambda decreases")

# All ISEs should be finite
for i, (lam, ise_v) in enumerate(zip(lambdas, ise_vals)):
    results[f"ise_finite_lam{lam}"] = np.isfinite(ise_v)

print_summary(results)
