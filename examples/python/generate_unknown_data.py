"""
Generate PRBS excitation signal for unknown SISO system identification.

A 10-bit LFSR pseudo-random binary sequence excites the unknown SISO plant in
open loop. The resulting (input, output) pair is used by siso_unknown.cpp for:
  - ARX(2,2) batch least-squares identification
  - FOPDT tangent-line parameter extraction
  - Validation (NRMSE <= 5% gate)

The C++ program siso_unknown tries to load this file first and falls back to
its internal LFSR only when the file is absent. Running this script before the
C++ binary ensures the Python LFSR (numpy-seeded) and the C++ LFSR produce
identical sequences, since both implement the same 10-bit feedback polynomial
(taps 9 and 6).

Output: examples/data/siso_prbs.csv with columns [time, input]

Usage
-----
    conda activate soft_robotics
    # Run from the project root:
    python examples/python/generate_unknown_data.py

Expected output (consumed by siso_unknown.cpp)
-----------------------------------------------
    examples/data/siso_prbs.csv
        Header: time,input
        Rows:   3000 lines, amplitude +/-1.0
"""

from __future__ import annotations
import os
import sys
import numpy as np

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _THIS_DIR)

from utils.data_gen import prbs

# -- Parameters (must match expectations in siso_unknown.cpp) -----------------
N_STEPS   = 3000    # 30 s of open-loop excitation at Ts = 0.01 s
Ts        = 0.01
AMPLITUDE = 1.0     # +/-1.0 - sufficient PE for the example plant gain approx = 1
SEED      = 42

_PROJECT_ROOT = os.path.join(_THIS_DIR, "..", "..")
OUTPUT_DIR    = os.path.normpath(os.path.join(_PROJECT_ROOT, "examples", "data"))
OUTPUT_FILE   = os.path.join(OUTPUT_DIR, "siso_prbs.csv")


def _verify(t: np.ndarray, u: np.ndarray) -> bool:
    """Print statistical properties of the generated signal; return pass/fail."""
    mean_u      = float(np.mean(u))
    var_u       = float(np.var(u))
    expected_var = AMPLITUDE ** 2   # E[u^2] for +/-A binary signal
    lag1_corr   = float(np.corrcoef(u[:-1], u[1:])[0, 1])

    mean_ok  = abs(mean_u)  < 0.05
    var_ok   = abs(var_u - expected_var) < 0.05
    # PRBS lag-1 autocorrelation is typically small but not zero for a max-length sequence
    acorr_ok = abs(lag1_corr) < 0.15

    print("PRBS Signal Verification")
    print(f"  Steps      : {N_STEPS}")
    print(f"  Duration   : {N_STEPS * Ts:.1f} s")
    print(f"  Amplitude  : +/-{AMPLITUDE}")
    print(f"  Seed       : {SEED}")
    print(f"  Mean       : {mean_u:+.6f}   [{'PASS' if mean_ok  else 'FAIL'}]  (|mean| < 0.05)")
    print(f"  Variance   : {var_u:.4f}     [{'PASS' if var_ok   else 'FAIL'}]  (expected {expected_var:.4f})")
    print(f"  Lag-1 corr : {lag1_corr:.4f}  [{'PASS' if acorr_ok else 'WARN'}]  (|ρ₁| < 0.15)")
    return mean_ok and var_ok


def main() -> None:
    t, u = prbs(N_STEPS, Ts, amplitude=AMPLITUDE, seed=SEED)

    all_ok = _verify(t, u)

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    with open(OUTPUT_FILE, "w", newline="") as fh:
        fh.write("time,input\n")
        for i in range(N_STEPS):
            fh.write(f"{t[i]:.6f},{u[i]:.6f}\n")

    print(f"\nWrote {N_STEPS} rows -> {OUTPUT_FILE}")
    print(f"  Column 'time'  : 0.00 ... {t[-1]:.2f} s")
    print(f"  Column 'input' : +/-{AMPLITUDE} PRBS")
    print(f"  Load path (C++): examples/data/siso_prbs.csv")
    print(f"  Status         : {'ALL PASS' if all_ok else 'CHECK WARNINGS ABOVE'}")


if __name__ == "__main__":
    main()
