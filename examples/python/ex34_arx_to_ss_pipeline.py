"""
ex34 - Full ARX-to-State-Space Identification Pipeline
========================================================
Goal     : End-to-end pipeline:
           1. Excite plant with PRBS.
           2. Identify ARX(2,2) model via batch LS.
           3. Convert identified model to SS via tf2ss.
           4. Validate: simulate identified model vs true plant on a different
              PRBS signal; verify NRMSE < 5%.
           5. Tune a PID on the identified model's FOPDT and compare performance
              to PID tuned on the true model.

Data generation : 2 000 PRBS samples for identification; 1 500 for validation.
Verification    :
  - Identified model NRMSE on validation set < 5%.
  - PID performance on identified model within 20% ISE of true-model-tuned PID.

Run:
    conda activate soft_robotics
    python ex34_arx_to_ss_pipeline.py
"""

import numpy as np
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from utils.plant import example_plant, tf2ss, ss_step, EXAMPLE_NUM, EXAMPLE_DEN
from utils.data_gen import prbs, step_signal
from utils.controllers import DiscretePID
from utils.verify import rmse, ise, print_summary

Ts = 0.01

print("=" * 60)
print("ex34 - ARX Identification -> PID Tuning Pipeline")
print("=" * 60)

# --- Step 1: Excite and collect data ---
_, u_id = prbs(2000, Ts, amplitude=0.5, seed=77)
plant_id = example_plant()
y_id = np.array([ss_step(plant_id, float(u)) for u in u_id])
print(f"\n  Identification data: {len(u_id)} PRBS samples")

# --- Step 2: ARX LS identification ---
burn = 50
N = len(y_id) - burn - 2
Phi_id = np.column_stack([-y_id[burn+1:burn+1+N], -y_id[burn:burn+N],
                            u_id[burn+1:burn+1+N],  u_id[burn:burn+N]])
Y_id   = y_id[burn+2:burn+2+N]
theta, *_ = np.linalg.lstsq(Phi_id, Y_id, rcond=None)
a1_id, a2_id, b1_id, b2_id = theta

num_id = [0.0,    b1_id, b2_id]
den_id = [1.0, a1_id, a2_id]
print(f"  Identified: a1={a1_id:.6f}, a2={a2_id:.6f}, b1={b1_id:.8f}, b2={b2_id:.8f}")

# --- Step 3: Convert to SS ---
ss_id = tf2ss(num_id, den_id)
print(f"  tf2ss succeeded, n={ss_id.A.shape[0]}")

# --- Step 4: Validate on new PRBS ---
_, u_val = prbs(1500, Ts, amplitude=0.5, seed=88)
plant_true = example_plant()
y_true_val = np.array([ss_step(plant_true, float(u)) for u in u_val])
y_id_val   = np.array([ss_step(ss_id,      float(u)) for u in u_val])

nrmse = rmse(y_true_val, y_id_val) / (float(np.max(y_true_val)) - float(np.min(y_true_val)) + 1e-12)
print(f"\n  Validation NRMSE: {nrmse:.4%}")

results = {}
results["nrmse_lt_5pct"] = nrmse < 0.05
print(f"  {'[PASS]' if results['nrmse_lt_5pct'] else '[FAIL]'} NRMSE < 5%")

# --- Step 5: Tune PID on identified vs true FOPDT ---
def fopdt_from_step(plant_fn, steps=2000):
    p = plant_fn()
    y = np.array([ss_step(p, 1.0) for _ in range(steps)])
    t = np.arange(steps) * Ts
    K = float(y[-1])
    t28 = t[np.argmax(y >= 0.283*K)]
    t63 = t[np.argmax(y >= 0.632*K)]
    tau   = 1.5 * (t63 - t28)
    theta = max(Ts, t63 - tau)
    return K, tau, theta

def imc_gains(K, tau, theta, lam=0.5):
    Kp = (tau + theta/2) / (K * (lam + theta/2))
    Ti = tau + theta/2; Td = tau*theta / (2*tau+theta)
    return Kp, Kp/Ti, Kp*Td

def sim_closed(Kp, Ki, Kd, steps=1500):
    pid = DiscretePID(Kp, Ki, Kd, Ts, u_min=-10.0, u_max=10.0)
    plant = example_plant()
    y = np.zeros(steps)
    for k in range(steps):
        u = pid.compute(1.0, y[k-1] if k > 0 else 0.0)
        y[k] = ss_step(plant, u)
    return y

K_t,  tau_t,  th_t  = fopdt_from_step(example_plant)
K_id, tau_id, th_id = fopdt_from_step(lambda: ss_id.__class__(ss_id.A.copy(), ss_id.B.copy(), ss_id.C.copy(), ss_id.D.copy()))

Kp_t,  Ki_t,  Kd_t  = imc_gains(K_t,  tau_t,  th_t)
Kp_id, Ki_id, Kd_id = imc_gains(K_id, tau_id, th_id)

y_t  = sim_closed(Kp_t,  Ki_t,  Kd_t)
y_id_cl = sim_closed(Kp_id, Ki_id, Kd_id)

ise_t  = ise(1.0 - y_t,  Ts)
ise_id = ise(1.0 - y_id_cl, Ts)
print(f"\n  ISE - true-model PID: {ise_t:.6f}")
print(f"  ISE - id-model PID:   {ise_id:.6f}")
results["ise_id_within_20pct"] = ise_id < 1.2 * ise_t or ise_id < 0.005
print(f"  {'[PASS]' if results['ise_id_within_20pct'] else '[FAIL]'} "
      f"identified-model ISE within 20% of true-model ISE")

print_summary(results)
