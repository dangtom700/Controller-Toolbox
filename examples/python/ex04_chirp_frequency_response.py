"""
ex04 - Chirp Excitation and Bode Plot Verification
===================================================
Audience : Experienced developers familiar with the C++ controller_toolbox.
Goal     : Drive the plant with a log-swept chirp, estimate the frequency
           response via Welch cross-spectral density, and verify that the
           magnitude and phase at the natural frequency match the analytic
           continuous-time Bode plot.

Data generation : 5 000 samples of chirp(0.01 -> 5 Hz) through ss_step().
Verification    :
  - |H(jomegan)| at omegan = 1 rad/s approx = 1/ζ_peak (for ζ=0.75, peak approx = 0 dB).
  - Phase at omega=0.1 rad/s approx = -arctan(1.5*0.1 / (1-0.01)) approx = -8.5^\circ.

Run:
    conda activate soft_robotics
    python ex04_chirp_frequency_response.py
"""

import numpy as np
from scipy import signal as sig
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from utils.plant import example_plant, ss_step
from utils.data_gen import chirp_signal
from utils.verify import assert_close, print_summary

Ts    = 0.01
STEPS = 5000

print("=" * 60)
print("ex04 - Chirp Excitation & Bode Verification")
print("=" * 60)

# --- Generate chirp and I/O data ---
t, u = chirp_signal(STEPS, Ts, f_start=0.01, f_end=5.0, amplitude=1.0)
plant = example_plant()
y = np.zeros(STEPS)
for k in range(STEPS):
    y[k] = ss_step(plant, u[k])

print(f"\nChirp: {STEPS} samples, 0.01-5.0 Hz, Ts={Ts} s")

# --- Frequency response estimate via cross-spectral density ---
fs = 1.0 / Ts
f_welch, Pyu = sig.csd(u, y, fs=fs, nperseg=512)
f_welch, Puu  = sig.welch(u, fs=fs, nperseg=512)

# Empirical frequency response H = Pyu / Puu
H_est = Pyu / (Puu + 1e-30)
mag_db_est = 20.0 * np.log10(np.abs(H_est) + 1e-30)
phase_deg_est = np.degrees(np.angle(H_est))

# --- Analytic Bode at selected frequencies ---
# G(s) = 1/(s^2+1.5s+1)
def G_analytic(omega):
    s = 1j * omega
    return 1.0 / (s**2 + 1.5*s + 1.0)

omega_check = np.array([0.1 * 2*np.pi,   # 0.1 rad/s
                         1.0,              # 1 rad/s (natural freq)
                         2.0 * 2*np.pi])  # 2 rad/s

print("\n  Frequency response verification:")
print(f"  {'omega (rad/s)':>12}  {'|H| analytic':>14}  {'|H| estimated':>14}  {'err':>8}")

results = {}
for omega in omega_check:
    G_an = G_analytic(omega)
    mag_an = 20.0 * np.log10(abs(G_an))
    # find nearest bin in Welch estimate
    f_hz = omega / (2 * np.pi)
    idx = np.argmin(np.abs(f_welch - f_hz))
    mag_est = mag_db_est[idx]
    err = abs(mag_est - mag_an)
    label = f"|H| at omega={omega:.2f}"
    ok = err < 2.0   # within 2 dB is acceptable for Welch estimate
    results[label] = ok
    tag = "[PASS]" if ok else "[FAIL]"
    print(f"  {omega:>12.3f}  {mag_an:>14.2f} dB  {mag_est:>14.2f} dB  {err:>7.2f} dB  {tag}")

# Phase at low frequency: should be near 0^\circ
idx_low = np.argmin(np.abs(f_welch - 0.1/(2*np.pi)))
phase_low = float(phase_deg_est[idx_low])
G_low = G_analytic(0.1)
phase_analytic = float(np.degrees(np.angle(G_low)))
print(f"\n  Phase at omega=0.1: estimated={phase_low:.1f}^\circ, analytic={phase_analytic:.1f}^\circ")
results["phase_low"] = abs(phase_low - phase_analytic) < 10.0
print(f"  {'[PASS]' if results['phase_low'] else '[FAIL]'} phase error < 10^\circ")

print_summary(results)
