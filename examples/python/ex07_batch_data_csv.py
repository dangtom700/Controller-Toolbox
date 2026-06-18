"""
ex07 - Batch Data Generation and CSV Round-Trip
================================================
Goal     : Generate a structured dataset (step + PRBS + chirp), save to CSV,
           reload, and verify every column survives the round-trip with zero
           error (no floating-point text truncation).

Data generation : 3 000 samples; three separate experiments concatenated.
Verification    : max |reloaded - original| < 1e-10 for all numeric columns.

Run:
    conda activate soft_robotics
    python ex07_batch_data_csv.py
"""

import numpy as np
import pandas as pd
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from utils.plant import example_plant, ss_step
from utils.data_gen import step_signal, prbs, chirp_signal
from utils.verify import check_alignment, assert_close, print_summary

Ts    = 0.01
N_SEG = 1000   # samples per segment

print("=" * 60)
print("ex07 - Batch Data CSV Round-Trip")
print("=" * 60)

def simulate(u_arr):
    plant = example_plant()
    return np.array([ss_step(plant, float(u)) for u in u_arr])

_, u_step  = step_signal(N_SEG, Ts)
_, u_prbs  = prbs(N_SEG, Ts, amplitude=0.5, seed=99)
_, u_chirp = chirp_signal(N_SEG, Ts, f_start=0.01, f_end=3.0)

y_step  = simulate(u_step)
y_prbs  = simulate(u_prbs)
y_chirp = simulate(u_chirp)

t = np.arange(3 * N_SEG) * Ts
u_all = np.concatenate([u_step, u_prbs, u_chirp])
y_all = np.concatenate([y_step, y_prbs, y_chirp])
exp_label = np.array(["step"]*N_SEG + ["prbs"]*N_SEG + ["chirp"]*N_SEG)

df_orig = pd.DataFrame({
    "time": t,
    "input": u_all,
    "output": y_all,
    "experiment": exp_label,
})

out_dir = os.path.join(os.path.dirname(__file__), "data")
os.makedirs(out_dir, exist_ok=True)
csv_path = os.path.join(out_dir, "batch_dataset.csv")
df_orig.to_csv(csv_path, index=False, float_format="%.15g")
print(f"\nSaved {len(df_orig)} rows -> {csv_path}")

df_load = pd.read_csv(csv_path)
print(f"Reloaded {len(df_load)} rows")

results = {}
results["row_count"] = len(df_orig) == len(df_load)
print(f"  {'[PASS]' if results['row_count'] else '[FAIL]'} row count matches")

for col in ["time", "input", "output"]:
    max_err = float(np.max(np.abs(df_orig[col].to_numpy() - df_load[col].to_numpy())))
    ok = max_err < 1e-10
    results[f"round_trip_{col}"] = ok
    print(f"  {'[PASS]' if ok else '[FAIL]'} {col}: max |err| = {max_err:.3e}")

results["labels_match"] = (df_orig["experiment"] == df_load["experiment"]).all()
print(f"  {'[PASS]' if results['labels_match'] else '[FAIL]'} experiment labels match")

# Verify segment boundaries
step_mask  = df_load["experiment"] == "step"
prbs_mask  = df_load["experiment"] == "prbs"
chirp_mask = df_load["experiment"] == "chirp"
results["segment_sizes"] = (step_mask.sum() == N_SEG and
                             prbs_mask.sum() == N_SEG and
                             chirp_mask.sum() == N_SEG)
print(f"  {'[PASS]' if results['segment_sizes'] else '[FAIL]'} each segment has {N_SEG} rows")

print_summary(results)
