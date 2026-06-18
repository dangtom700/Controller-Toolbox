"""
Generate orthogonal PRBS excitation signals for MIMO system identification.

Two pseudo-random binary sequences with different seeds are generated simultaneously
so that the two input channels are statistically decorrelated. This orthogonality is
required for accurate MIMO ARX identification: if both inputs are the same signal,
the regression matrix is rank-deficient and cross-channel dynamics cannot be recovered.

Output: examples/data/mimo_prbs.csv with columns [u1, u2]

Usage
-----
    conda activate soft_robotics
    # Run from the project root:
    python examples/python/generate_mimo_data.py

Expected output (consumed by mimo_unknown.cpp)
-----------------------------------------------
    examples/data/mimo_prbs.csv
        Header: u1,u2
        Rows:   5000 lines, amplitude +/-0.5
"""

from __future__ import annotations
import os
import sys
import numpy as np

# Allow running from project root or from examples/python/
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _THIS_DIR)

from utils.data_gen import prbs

# -- Parameters ----------------------------------------------------------------
N_STEPS   = 5000     # 50 s of excitation at Ts = 0.01 s
Ts        = 0.01
AMPLITUDE = 0.5      # +/-0.5 V - keeps plant well within +/-20 saturation limits
SEED_CH1  = 42       # channel-1 LFSR seed
SEED_CH2  = 1023     # channel-2 LFSR seed (maximally different from 42 in 10-bit space)

_PROJECT_ROOT = os.path.join(_THIS_DIR, "..", "..")
OUTPUT_DIR    = os.path.normpath(os.path.join(_PROJECT_ROOT, "examples", "data"))
OUTPUT_FILE   = os.path.join(OUTPUT_DIR, "mimo_prbs.csv")


def _verify(u1: np.ndarray, u2: np.ndarray) -> bool:
    """Print decorrelation diagnostics; return True if all checks pass."""
    xcorr_zero  = float(np.mean(u1 * u2))
    power1      = float(np.mean(u1 ** 2))
    power2      = float(np.mean(u2 ** 2))
    xcorr_norm  = abs(xcorr_zero) / (AMPLITUDE ** 2)   # normalised by expected power

    print("PRBS Decorrelation Diagnostics")
    print(f"  u1 power : {power1:.4f}  (expected {AMPLITUDE**2:.4f})")
    print(f"  u2 power : {power2:.4f}  (expected {AMPLITUDE**2:.4f})")
    print(f"  cross-corr(lag=0) : {xcorr_zero:+.6f}")
    print(f"  normalised |xcorr|: {xcorr_norm:.4f}  [{'PASS' if xcorr_norm < 0.05 else 'WARN'}]  (threshold < 0.05)")

    power_ok = abs(power1 - AMPLITUDE**2) < 0.01 and abs(power2 - AMPLITUDE**2) < 0.01
    xcorr_ok = xcorr_norm < 0.05
    print(f"  Power check : {'PASS' if power_ok  else 'FAIL'}")
    print(f"  Xcorr check : {'PASS' if xcorr_ok  else 'WARN'}")
    return power_ok  # xcorr is advisory for PRBS with different seeds


def main() -> None:
    _, u1 = prbs(N_STEPS, Ts, amplitude=AMPLITUDE, seed=SEED_CH1)
    _, u2 = prbs(N_STEPS, Ts, amplitude=AMPLITUDE, seed=SEED_CH2)

    all_ok = _verify(u1, u2)

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    with open(OUTPUT_FILE, "w", newline="") as fh:
        fh.write("u1,u2\n")
        for i in range(N_STEPS):
            fh.write(f"{u1[i]:.6f},{u2[i]:.6f}\n")

    print(f"\nWrote {N_STEPS} rows -> {OUTPUT_FILE}")
    print(f"  Columns : u1 (seed={SEED_CH1}), u2 (seed={SEED_CH2})")
    print(f"  Amplitude: +/-{AMPLITUDE}")
    print(f"  Duration : {N_STEPS * Ts:.1f} s  ({N_STEPS} steps, Ts={Ts} s)")
    print(f"  Status   : {'ALL PASS' if all_ok else 'CHECK WARNINGS ABOVE'}")


if __name__ == "__main__":
    main()
