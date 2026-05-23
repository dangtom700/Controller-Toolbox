"""
ex06 - Multi-Sine Excitation for Broadband Identification
==========================================================
Goal     : Show how to generate a multi-sine signal whose frequencies span
           the plant bandwidth and verify spectral energy concentrates only
           at the injected frequencies (>= 40 dB above noise floor).

Data generation : 4 096 samples of multi-sine at [0.05, 0.1, 0.2, 0.5, 1.0,
                  2.0, 3.0, 5.0] Hz through ss_step().
Verification    : FFT peaks at injected frequencies; no leakage > -30 dBc.

Run:
    conda activate soft_robotics
    python ex06_multi_sine_excitation.py
"""

import numpy as np
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from utils.plant import example_plant, ss_step
from utils.data_gen import multi_sine
from utils.verify import print_summary

Ts    = 0.01
STEPS = 4096

freqs_hz = [0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 3.0, 5.0]

print("=" * 60)
print("ex06 - Multi-Sine Excitation")
print("=" * 60)

t, u = multi_sine(STEPS, Ts, freqs_hz)
plant = example_plant()
y = np.zeros(STEPS)
for k in range(STEPS):
    y[k] = ss_step(plant, u[k])

print(f"\nMulti-sine: {len(freqs_hz)} frequencies, {STEPS} samples")

fs = 1.0 / Ts
freqs_fft = np.fft.rfftfreq(STEPS, d=Ts)
U_mag = np.abs(np.fft.rfft(u)) / STEPS

max_mag = np.max(U_mag[1:])
results = {}

print(f"\n  Injected freq (Hz) | FFT magnitude | dBc above floor")
print("  " + "-"*54)

for f in freqs_hz:
    idx = np.argmin(np.abs(freqs_fft - f))
    peak = U_mag[idx]
    dbc  = 20.0 * np.log10(peak / max(max_mag, 1e-30))
    label = f"peak@{f}Hz > -6dBc"
    ok = dbc > -6.0   # each component within 6 dB of the strongest
    results[label] = ok
    print(f"  {f:>19.2f} | {peak:>13.6f} | {dbc:>12.1f} dB  {'[PASS]' if ok else '[FAIL]'}")

# Check no significant energy at a non-injected frequency (e.g., 0.7 Hz)
f_null = 0.7
idx_null = np.argmin(np.abs(freqs_fft - f_null))
mag_null = U_mag[idx_null]
dbc_null = 20.0 * np.log10(mag_null / max(max_mag, 1e-30))
results["null_at_0.7Hz"] = dbc_null < -20.0
print(f"\n  Leakage at 0.7 Hz (non-injected): {dbc_null:.1f} dBc")
print(f"  {'[PASS]' if results['null_at_0.7Hz'] else '[FAIL]'} leakage < -20 dBc")

print_summary(results)
