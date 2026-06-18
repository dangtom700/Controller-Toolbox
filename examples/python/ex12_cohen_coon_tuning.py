"""
ex12 - Cohen-Coon PID Tuning
==============================
Goal     : Apply the Cohen-Coon tuning rule to the FOPDT model and compare
           the resulting ISE/ITAE against the IMC-tuned PID from ex10.

Data generation : FOPDT from step response; 2 000-sample closed-loop sims.
Verification    :
  - Cohen-Coon gains differ from IMC gains (they use different design specs).
  - Both PIDs stabilise the plant (finite, bounded output).

Run:
    conda activate soft_robotics
    python ex12_cohen_coon_tuning.py
"""

import numpy as np
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from utils.plant import example_plant, ss_step
from utils.controllers import DiscretePID
from utils.verify import ise, itae, print_summary, assert_close

Ts    = 0.01
STEPS = 2000

print("=" * 60)
print("ex12 - Cohen-Coon PID Tuning")
print("=" * 60)

# --- FOPDT identification ---
plant_id = example_plant()
y_step = np.array([ss_step(plant_id, 1.0) for _ in range(STEPS)])
t = np.arange(STEPS) * Ts

K = float(y_step[-1])
t28 = t[np.argmax(y_step >= 0.283 * K)]
t63 = t[np.argmax(y_step >= 0.632 * K)]
tau   = 1.5 * (t63 - t28)
theta = max(Ts, t63 - tau)

print(f"\n  FOPDT: K={K:.4f}, tau={tau:.4f} s, theta={theta:.4f} s")

# --- Cohen-Coon PID formulas ---
r = theta / tau   # delay ratio
Kp_cc = (1.0 / K) * (tau / theta) * (1.33 + r / 4.0)
Ti_cc = theta * (30.0 + 3.0*r) / (9.0 + 20.0*r)
Td_cc = theta * 4.0 / (11.0 + 2.0*r)
Ki_cc = Kp_cc / Ti_cc
Kd_cc = Kp_cc * Td_cc

print(f"\n  Cohen-Coon: Kp={Kp_cc:.4f}, Ki={Ki_cc:.4f}, Kd={Kd_cc:.4f}")

# --- IMC for comparison (lambda = tau/2) ---
lam = tau / 2.0
Kp_imc = (tau + theta/2.0) / (K * (lam + theta/2.0))
Ti_imc = tau + theta / 2.0
Td_imc = tau * theta / (2.0 * tau + theta)
Ki_imc = Kp_imc / Ti_imc
Kd_imc = Kp_imc * Td_imc

print(f"  IMC (lambda={lam:.2f}): Kp={Kp_imc:.4f}, Ki={Ki_imc:.4f}, Kd={Kd_imc:.4f}")

def sim_closed(Kp, Ki, Kd):
    pid = DiscretePID(Kp=Kp, Ki=Ki, Kd=Kd, Ts=Ts, u_min=-10.0, u_max=10.0)
    plant = example_plant()
    y = np.zeros(STEPS)
    for k in range(STEPS):
        u = pid.compute(1.0, y[k-1] if k > 0 else 0.0)
        y[k] = ss_step(plant, u)
    return y

y_cc  = sim_closed(Kp_cc,  Ki_cc,  Kd_cc)
y_imc = sim_closed(Kp_imc, Ki_imc, Kd_imc)

err_cc  = 1.0 - y_cc
err_imc = 1.0 - y_imc

ise_cc,  itae_cc  = ise(err_cc,  Ts), itae(err_cc,  Ts)
ise_imc, itae_imc = ise(err_imc, Ts), itae(err_imc, Ts)

print(f"\n  Metric  | Cohen-Coon | IMC")
print(f"  ISE     | {ise_cc:>10.5f} | {ise_imc:>10.5f}")
print(f"  ITAE    | {itae_cc:>10.5f} | {itae_imc:>10.5f}")

results = {}
results["cc_stable"]  = np.all(np.isfinite(y_cc))  and float(np.max(np.abs(y_cc)))  < 50.0
results["imc_stable"] = np.all(np.isfinite(y_imc)) and float(np.max(np.abs(y_imc))) < 50.0
print(f"\n  {'[PASS]' if results['cc_stable']  else '[FAIL]'} Cohen-Coon stable")
print(f"  {'[PASS]' if results['imc_stable'] else '[FAIL]'} IMC stable")

results["gains_differ"] = abs(Kp_cc - Kp_imc) > 0.01
print(f"  {'[PASS]' if results['gains_differ'] else '[FAIL]'} "
      f"CC and IMC Kp differ (|{Kp_cc:.4f} - {Kp_imc:.4f}| = {abs(Kp_cc-Kp_imc):.4f})")

print_summary(results)
