"""
ex87_l1_adaptive.py -- L1 Adaptive Control demo.

Demonstrates:
  1. L1 adaptive controller tracking a step reference on a first-order plant.
  2. Robustness to sudden gain mismatch (gain doubles at k=200).
  3. Comparison with fixed-gain MRAC (no LP filter) under same mismatch.

Plant:  y[k+1] = a*y[k] + b*u[k],   a=0.85, b=0.15 (nominal)
        At k=200: b doubles to 0.30  (unmodelled gain change)
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'build', 'bindings'))
if sys.platform == "win32" and hasattr(os, "add_dll_directory"):
    for _p in [r"C:\msys64\mingw64\bin"]:
        if os.path.isdir(_p):
            os.add_dll_directory(_p)

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'L1AdaptiveController'):
        raise AttributeError("L1AdaptiveController not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

import numpy as np

print("=== ex87: L1 Adaptive Control ===")

TS = 0.01
N  = 400
R  = 1.0

# ---------------------------------------------------------------------------
# L1 Adaptive Controller
# ---------------------------------------------------------------------------
p = ctrl.L1AdaptiveParams()
p.a_m      = 0.85
p.b_m      = 0.15
p.k_g      = 1.0
p.Gamma    = 300.0
p.omega_c  = 15.0
p.sigma_max = 20.0
p.uMin     = -5.0
p.uMax     =  5.0

l1 = ctrl.L1AdaptiveController(p, TS)

y_l1  = 0.0
iae_l1_1 = 0.0  # first half
iae_l1_2 = 0.0  # second half (gain mismatch)

print(f"\n{'k':>5}  {'y_L1':>8}  {'sigma_hat':>10}  {'note'}")
for k in range(N):
    b_actual = 0.15 if k < 200 else 0.30  # gain doubles at k=200

    l1.set_reference(R)
    u = l1.compute(y_l1)
    y_l1 = 0.85 * y_l1 + b_actual * u

    if k < 200:
        iae_l1_1 += abs(R - y_l1) * TS
    else:
        iae_l1_2 += abs(R - y_l1) * TS

    if k in (50, 199, 200, 250, 399):
        note = "gain change" if k == 200 else ""
        print(f"  {k:4d}  {y_l1:8.4f}  {l1.estimated_disturbance():10.4f}  {note}")

print(f"\n  IAE phase-1 (k=0..199):   {iae_l1_1:.4f}")
print(f"  IAE phase-2 (k=200..399): {iae_l1_2:.4f}  (gain doubled at k=200)")
print(f"  Final y: {y_l1:.4f}  (target: {R:.1f})")

# ---------------------------------------------------------------------------
# Compare: MRAC (no LP filter, same adaptation gain) - may oscillate
# ---------------------------------------------------------------------------
print("\n-- MRAC comparison (MRACController) --")
try:
    if not hasattr(ctrl, 'MRACController'):
        raise AttributeError()
    mp = ctrl.MRACParams()
    mp.a_m    = 0.85
    mp.b_m    = 0.15
    mp.Gamma  = 50.0   # must be lower to stay stable without LP filter
    mp.k_g    = 1.0
    mp.uMin   = -5.0
    mp.uMax   =  5.0
    mrac = ctrl.MRACController(mp, TS)

    y_mrac = 0.0
    iae_mrac_2 = 0.0
    for k in range(N):
        b_actual = 0.15 if k < 200 else 0.30
        mrac.set_reference(R)
        u_m = mrac.compute(y_mrac)
        y_mrac = 0.85 * y_mrac + b_actual * u_m
        if k >= 200:
            iae_mrac_2 += abs(R - y_mrac) * TS

    print(f"  MRAC IAE phase-2:  {iae_mrac_2:.4f}")
    print(f"  L1   IAE phase-2:  {iae_l1_2:.4f}  (should be comparable or lower)")
except AttributeError:
    print("  (MRACController not found - skipping comparison)")

print("\n[PASS] L1AdaptiveController demo complete.")
