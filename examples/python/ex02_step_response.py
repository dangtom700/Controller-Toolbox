"""
ex02 - Step Response Simulation and CSV Cross-Validation
=========================================================
Audience : Experienced developers familiar with the C++ controller_toolbox.
Goal     : Simulate the open-loop step response of the example plant and
           cross-validate sample-by-sample against the CSV produced by the
           C++ generate_test_data.py script (tests/data/step_response.csv).

Data generation : 1 500 step of unit step through ss_step() - same math as
                  the existing scripts/generate_test_data.py.
Verification    : max absolute deviation vs CSV must be < 1e-9.

Run:
    conda activate soft_robotics
    python ex02_step_response.py
"""

import numpy as np
import pandas as pd
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from utils.plant import example_plant, ss_step
from utils.verify import check_alignment, dc_gain_check, assert_close, print_summary

Ts    = 0.01
STEPS = 1500

print("=" * 60)
print("ex02 - Step Response Simulation")
print("=" * 60)

# --- Simulate ---
plant = example_plant()
t_sim = np.arange(STEPS) * Ts
y_sim = np.zeros(STEPS)

for k in range(STEPS):
    y_sim[k] = ss_step(plant, 1.0)   # unit step

print(f"\nSimulated {STEPS} steps at Ts={Ts} s")
print(f"  y[0]   = {y_sim[0]:.9f}   (expected 0.0)")
print(f"  y[-1]  = {y_sim[-1]:.9f}  (expected approx = 1.0)")

# --- Cross-validate against CSV (if present) ---
csv_path = os.path.join(os.path.dirname(__file__),
                        "../../tests/data/step_response.csv")
csv_path = os.path.normpath(csv_path)
results = {}

if os.path.exists(csv_path):
    df = pd.read_csv(csv_path)
    t_csv = df["time"].to_numpy()
    y_csv = df["output"].to_numpy()

    results["alignment"] = check_alignment(t_sim, y_sim, t_csv, y_csv,
                                           labels=["t_sim","y_sim","t_csv","y_csv"])
    max_dev = float(np.max(np.abs(y_sim - y_csv)))
    results["csv_match"] = assert_close(max_dev, 0.0, tol=1e-9,
                                        label="max |y_sim - y_csv|")
    print(f"\n  CSV loaded: {csv_path}")
    print(f"  Max deviation from CSV: {max_dev:.3e}")
else:
    print(f"\n  [SKIP] CSV not found at {csv_path}")
    print("  Run: python scripts/generate_test_data.py  to generate it first.")
    results["alignment"] = True   # can't check without file

# --- DC gain check ---
results["dc_gain"] = dc_gain_check(y_sim, expected=1.0, tol=0.01)

# --- FOPDT tangent method (28.3% / 63.2% crossings) ---
K   = float(y_sim[-1])
y28 = 0.283 * K
y63 = 0.632 * K
t28 = t_sim[np.argmax(y_sim >= y28)]
t63 = t_sim[np.argmax(y_sim >= y63)]
tau   = 1.5 * (t63 - t28)
theta = max(0.0, t63 - tau)

print(f"\n  FOPDT estimate (tangent method):")
print(f"    K     = {K:.4f}")
print(f"    tau   = {tau:.4f} s")
print(f"    theta = {theta:.4f} s")

# For this 2nd-order plant, a rough sanity: tau should be O(1)
results["tau_sanity"] = assert_close(tau, 1.0, tol=0.5, label="tau approx = 1.0 s")

print_summary(results)
