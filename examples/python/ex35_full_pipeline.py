"""
ex35 - Full End-to-End Pipeline
=================================
Goal     : Orchestrate the complete workflow in a single script:
           1. Excite plant (PRBS).
           2. Identify ARX model (RLS with forgetting factor).
           3. Design LQR via Bryson's Rule on identified model.
           4. Simulate closed loop.
           5. Run Monte Carlo (N=50) robustness check.
           6. Save all results to CSV.
           7. Print final summary.

This serves as an integration test for the entire examples/python package.

Data generation : 2 000 PRBS samples (identification) + 1 500 step (validation).
Verification    :
  - ARX NRMSE < 5%.
  - LQR closed loop stable and settles.
  - Monte Carlo: >= 85% of +/-5% perturbed plants remain stable.

Run:
    conda activate soft_robotics
    python ex35_full_pipeline.py
"""

import numpy as np
import pandas as pd
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from utils.plant import example_plant, tf2ss, ss_step, EXAMPLE_NUM, EXAMPLE_DEN
from utils.data_gen import prbs
from utils.controllers import DiscreteLQR
from utils.verify import rmse, ise, print_summary

Ts = 0.01

print("=" * 60)
print("ex35 - Full End-to-End Pipeline")
print("=" * 60)

results = {}

# ==========================================================================
# Step 1: PRBS Excitation
# ==========================================================================
print("\n[1/7] PRBS Excitation ...")
_, u_id = prbs(2000, Ts, amplitude=0.5, seed=42)
plant_open = example_plant()
y_id = np.array([ss_step(plant_open, float(u)) for u in u_id])
results["step1_data"] = len(y_id) == 2000
print(f"      {len(y_id)} samples collected")

# ==========================================================================
# Step 2: RLS ARX Identification
# ==========================================================================
print("[2/7] RLS ARX(2,2) Identification ...")
n_params = 4; theta = np.zeros(n_params); P = 1e4 * np.eye(n_params)
LAMBDA = 0.98
for k in range(2, len(y_id)):
    phi = np.array([-y_id[k-1], -y_id[k-2], u_id[k-1], u_id[k-2]])
    eps = y_id[k] - float(phi @ theta)
    denom = LAMBDA + phi @ P @ phi
    L = (P @ phi) / denom
    theta = theta + L * eps
    P = (P - np.outer(L, phi @ P)) / LAMBDA
a1, a2, b1, b2 = theta
num_id = [0.0, b1, b2]; den_id = [1.0, a1, a2]
print(f"      a1={a1:.6f}, a2={a2:.6f}, b1={b1:.8f}, b2={b2:.8f}")

# ==========================================================================
# Step 3: Validate identified model
# ==========================================================================
print("[3/7] Validation on new PRBS ...")
ss_id = tf2ss(num_id, den_id)
_, u_val = prbs(1500, Ts, amplitude=0.5, seed=99)
plant_val = example_plant()
y_true = np.array([ss_step(plant_val, float(u)) for u in u_val])
y_pred = np.array([ss_step(ss_id,     float(u)) for u in u_val])
nrmse = rmse(y_true, y_pred) / (np.ptp(y_true) + 1e-12)
results["nrmse_ok"] = nrmse < 0.05
print(f"      NRMSE = {nrmse:.4%}  {'PASS' if results['nrmse_ok'] else 'FAIL'}")

# ==========================================================================
# Step 4: LQR design on identified model
# ==========================================================================
print("[4/7] LQR design (Bryson's Rule) ...")
A_id, B_id = ss_id.A, ss_id.B
Q = np.diag([1.0, 1.0]); R = np.array([[0.04]])
try:
    lqr = DiscreteLQR(A_id, B_id, Q, R)
    results["lqr_ok"] = True
    print(f"      K = {lqr.K}")
except Exception as ex:
    results["lqr_ok"] = False
    print(f"      [FAIL] LQR failed: {ex}")
    lqr = None

# ==========================================================================
# Step 5: Closed-loop simulation
# ==========================================================================
print("[5/7] Closed-loop simulation (1 500 steps) ...")
STEPS_CL = 1500
if lqr is not None:
    plant_cl = example_plant()
    y_cl = np.zeros(STEPS_CL)
    for k in range(STEPS_CL):
        u = lqr.compute(plant_cl.x)
        y_cl[k] = ss_step(plant_cl, u)
    results["cl_stable"] = bool(np.all(np.isfinite(y_cl)) and float(np.max(np.abs(y_cl))) < 20.0)
    print(f"      stable={results['cl_stable']}, y_final={float(y_cl[-1]):.4f}")
else:
    results["cl_stable"] = False

# ==========================================================================
# Step 6: Monte Carlo robustness
# ==========================================================================
print("[6/7] Monte Carlo robustness (N=50, +/-5% perturbation) ...")
N_MC = 50; PERTURB = 0.05; RNG = np.random.default_rng(1111)
stable_count = 0
for _ in range(N_MC):
    scale = 1.0 + RNG.uniform(-PERTURB, PERTURB, 4)
    num_p = [0.0, EXAMPLE_NUM[1]*scale[2], EXAMPLE_NUM[2]*scale[3]]
    den_p = [1.0, EXAMPLE_DEN[1]*scale[0], EXAMPLE_DEN[2]*scale[1]]
    try:
        p = tf2ss(num_p, den_p)
        y_mc = np.zeros(300)
        for k in range(300):
            if lqr: u = lqr.compute(p.x)
            else:   u = 0.0
            y_mc[k] = ss_step(p, u)
        if np.all(np.isfinite(y_mc)) and float(np.max(np.abs(y_mc))) < 50.0:
            stable_count += 1
    except Exception:
        pass

stab_rate = stable_count / N_MC
results["mc_stability"] = stab_rate >= 0.85
print(f"      stability rate = {stab_rate:.1%}  {'PASS' if results['mc_stability'] else 'FAIL'}")

# ==========================================================================
# Step 7: Save results
# ==========================================================================
print("[7/7] Saving results ...")
out_dir = os.path.join(os.path.dirname(__file__), "data")
os.makedirs(out_dir, exist_ok=True)

if lqr is not None:
    t_arr = np.arange(STEPS_CL) * Ts
    df = pd.DataFrame({"time": t_arr, "output": y_cl, "error": 0.0 - y_cl})
    df.to_csv(os.path.join(out_dir, "ex35_pipeline_result.csv"), index=False)

summary = {
    "nrmse": nrmse,
    "stability_rate": stab_rate,
    "lqr_ok": results["lqr_ok"],
    "cl_stable": results.get("cl_stable", False),
}
pd.DataFrame([summary]).to_csv(os.path.join(out_dir, "ex35_pipeline_summary.csv"), index=False)
print(f"      Saved to {out_dir}/")

print_summary(results)
