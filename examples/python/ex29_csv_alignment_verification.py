"""
ex29 - CSV Alignment and Timestamp Verification
=================================================
Goal     : Given two CSVs produced by different scripts (simulate_all.cpp and
           realtime_all.cpp), verify that the time columns are aligned,
           monotonically increasing, uniformly spaced, and that no row is
           missing by comparing row counts.

Data generation : Synthetic CSV files created programmatically to mimic the
                  format output by scripts/simulate_all.cpp.
Verification    :
  - Time column is monotonically increasing.
  - Time steps are uniform: max|Δt - Ts| / Ts < 0.1%.
  - All numeric columns contain no NaN/Inf.
  - Row count matches expected STEPS.

Run:
    conda activate soft_robotics
    python ex29_csv_alignment_verification.py
"""

import numpy as np
import pandas as pd
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from utils.plant import example_plant, ss_step
from utils.controllers import DiscretePID
from utils.verify import check_alignment, print_summary

Ts    = 0.01
STEPS = 1500

print("=" * 60)
print("ex29 - CSV Alignment Verification")
print("=" * 60)

# --- Generate synthetic CSV (mimics simulate_all.cpp output) ---
pid   = DiscretePID(Kp=3.0, Ki=1.5, Kd=0.75, Ts=Ts, u_min=-10.0, u_max=10.0)
plant = example_plant()

t_arr = np.arange(STEPS) * Ts
r_arr = np.ones(STEPS)
y_arr = np.zeros(STEPS)
u_arr = np.zeros(STEPS)
e_arr = np.zeros(STEPS)

for k in range(STEPS):
    u_arr[k] = pid.compute(1.0, y_arr[k-1] if k > 0 else 0.0)
    y_arr[k] = ss_step(plant, u_arr[k])
    e_arr[k] = r_arr[k] - y_arr[k]

out_dir = os.path.join(os.path.dirname(__file__), "data")
os.makedirs(out_dir, exist_ok=True)
csv_path = os.path.join(out_dir, "sim_pid_verification.csv")

df = pd.DataFrame({
    "time":      t_arr,
    "reference": r_arr,
    "output":    y_arr,
    "control":   u_arr,
    "error":     e_arr,
})
df.to_csv(csv_path, index=False, float_format="%.9f")
print(f"\nWrote {len(df)} rows -> {csv_path}")

# --- Verification suite ---
df_load = pd.read_csv(csv_path)
results = {}

# Row count
results["row_count"] = len(df_load) == STEPS
print(f"\n  {'[PASS]' if results['row_count'] else '[FAIL]'} "
      f"row count: {len(df_load)} == {STEPS}")

t = df_load["time"].to_numpy()

# Monotonically increasing
diffs = np.diff(t)
results["monotone"] = bool(np.all(diffs > 0))
print(f"  {'[PASS]' if results['monotone'] else '[FAIL]'} time monotonically increasing")

# Uniform spacing
dt_mean = float(np.mean(diffs))
dt_max_err = float(np.max(np.abs(diffs - Ts)) / Ts)
results["uniform"] = dt_max_err < 0.001
print(f"  {'[PASS]' if results['uniform'] else '[FAIL]'} "
      f"uniform spacing: max Δt error = {dt_max_err:.2e} ({dt_max_err:.4%})")

# No NaN / Inf
for col in ["time", "reference", "output", "control", "error"]:
    ok = bool(np.all(np.isfinite(df_load[col].to_numpy())))
    results[f"finite_{col}"] = ok
    if not ok:
        print(f"  [FAIL] column '{col}' contains NaN/Inf")
print(f"  {'[PASS]' if all(results[f'finite_{c}'] for c in ['time','output','control','error']) else '[FAIL]'} "
      "all columns finite")

# Column alignment
results["columns_aligned"] = check_alignment(
    t, df_load["output"].to_numpy(), df_load["control"].to_numpy(),
    labels=["time", "output", "control"]
)

print_summary(results)
