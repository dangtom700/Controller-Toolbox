"""
ex105_mu_analysis.py

Robustness Phase 3: structured singular value (mu) D-scaling analysis.

Two checks against analytically-known values:
  1. A classic 2x2 off-diagonal example (Skogestad & Postlethwaite) with two
     independent SISO complex-full uncertainty blocks: mu = sqrt(|m12*m21|) = 1,
     strictly below the unstructured sigma_max(M) = 2.
  2. The Gang-of-Four fixture from ex104: peak mu of the closed loop under scaled
     output multiplicative uncertainty M(jw) = sigma_rel * T(jw), for a single
     ComplexFull block spanning the SISO output, reduces exactly to
     sigma_rel * ||T||_inf = sigma_rel / 3. The robust stability radius
     (largest sigma_rel keeping peak mu < 1) is therefore 3.
"""

import sys
import os
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    for name in ('UncertaintyStructure', 'UncertaintyBlock', 'MuBound',
                 'PeakMuResult', 'compute_mu', 'peak_mu', 'robust_stability_radius'):
        if not hasattr(ctrl, name):
            raise AttributeError(f"{name} not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)


print("=== Robustness Phase 3: Structured Singular Value (mu) Analysis ===")

ok = True

# ---- Part 1: classic 2x2 off-diagonal example --------------------------------
block_a = ctrl.UncertaintyBlock()
block_a.type = ctrl.UncertaintyBlock.Type.ComplexFull
block_a.r_out, block_a.r_in = 1, 1
block_b = ctrl.UncertaintyBlock()
block_b.type = ctrl.UncertaintyBlock.Type.ComplexFull
block_b.r_out, block_b.r_in = 1, 1

struc2 = ctrl.UncertaintyStructure()
struc2.blocks = [block_a, block_b]

M = np.array([[0.0, 2.0],
              [0.5, 0.0]], dtype=complex)

bounds = ctrl.compute_mu([M], struc2, True)
mu2 = bounds[0]

print("\n-- Classic 2x2 off-diagonal example --")
print("  sigma_max(M) (unstructured) = 2.0000")
print(f"  mu upper bound (D-scaled)   = {mu2.upper:.4f}")
print(f"  mu lower bound (rho(M))     = {mu2.lower:.4f}")

if abs(mu2.upper - 1.0) > 1e-3 or abs(mu2.lower - 1.0) > 1e-9:
    print("[FAIL] mu should equal 1, strictly tighter than sigma_max=2")
    ok = False
else:
    print("  [mu=1, tighter than sigma_max] PASS")

# ---- Part 2: closed-loop peak mu + robust stability radius ------------------
Ts = 0.1
G = ctrl.StateSpace(np.array([[0.6]]), np.array([[0.4]]),
                    np.array([[1.0]]), np.array([[0.0]]), Ts)
# Static-gain controller as a 1-state realisation of D = 0.5.
K = ctrl.StateSpace(np.zeros((1, 1)), np.zeros((1, 1)),
                    np.zeros((1, 1)), np.array([[0.5]]), Ts)

block_full = ctrl.UncertaintyBlock()
block_full.type = ctrl.UncertaintyBlock.Type.ComplexFull
block_full.r_out, block_full.r_in = 1, 1
struc1 = ctrl.UncertaintyStructure()
struc1.blocks = [block_full]

peak = ctrl.peak_mu(G, K, struc1, sigma_rel=1.0, freq_points=50, omega_min=1e-4)
radius = ctrl.robust_stability_radius(G, K, struc1, sigma_max=5.0)

print("\n-- Closed-loop peak mu (single ComplexFull block) --")
print(f"  peak mu @ sigma_rel=1 (== ||T||_inf) = {peak.peak.upper:.4f}")
print(f"  peak frequency [rad/s]                = {peak.peak_omega_rad_s:.6f}")
print(f"  robust stability radius (1/||T||_inf) = {radius:.4f}")

if abs(peak.peak.upper - 1.0 / 3.0) > 1e-3:
    print("[FAIL] peak mu should equal ||T||_inf = 1/3")
    ok = False
else:
    print("  [peak == ||T||_inf]      PASS")

if abs(radius - 3.0) > 1e-2:
    print("[FAIL] robust stability radius should equal 1/||T||_inf = 3.0")
    ok = False
else:
    print("  [radius == 1/||T||_inf]  PASS")

print("\n[PASS] All checks passed." if ok else "\n[FAIL] One or more checks failed.")
