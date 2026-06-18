"""
ex104_gang_of_four.py

Robustness Phase 2: Gang of Four + Disk Margin via SystemAnalysis block algebra.

Builds the closed-loop Gang of Four (S, T, GS, KS) for a unity-feedback loop using
the series()/parallel()/feedback() block-algebra primitives, then derives the
simultaneous gain/phase disk margin from the same loop.

    Plant:      x[k+1] = 0.6 x[k] + 0.4 u[k],  y = x   (open-loop stable)
    Controller: u = 0.5 * e   (static gain; closed-loop pole 0.4)
"""

import sys
import os
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    for name in ('DiskMargin', 'GangOfFour', 'GangOfFourNorms', 'SystemAnalysis'):
        if not hasattr(ctrl, name):
            raise AttributeError(f"{name} not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)


Ts = 0.1
G = ctrl.StateSpace(np.array([[0.6]]), np.array([[0.4]]),
                    np.array([[1.0]]), np.array([[0.0]]), Ts)
# Static-gain controller as a 1-state realisation of D = 0.5.
K = ctrl.StateSpace(np.zeros((1, 1)), np.zeros((1, 1)),
                    np.zeros((1, 1)), np.array([[0.5]]), Ts)

print("=== Robustness Phase 2: Gang of Four + Disk Margin ===")

g4 = ctrl.SystemAnalysis.gang_of_four(G, K)
g4n = ctrl.SystemAnalysis.gang_of_four_norms(g4)

print(f"  ||S||_inf  = {g4n.norm_S:.4f}")
print(f"  ||T||_inf  = {g4n.norm_T:.4f}")
print(f"  ||GS||_inf = {g4n.norm_GS:.4f}")
print(f"  ||KS||_inf = {g4n.norm_KS:.4f}")

L = ctrl.SystemAnalysis.series(K, G)
dm = ctrl.SystemAnalysis.calculate_disk_margin(L)

print(f"  disk margin alpha        = {dm.alpha:.4f}")
print(f"  simultaneous gain margin = {dm.gain_margin:.4f} (linear)")
print(f"  simultaneous phase margin = {dm.phase_margin_deg:.4f} deg")

# ---- Internal consistency checks (printed, do not abort the example) --------
ok = True

freqs = list(np.linspace(0.5, 25.0, 8))
S_resp = np.array(ctrl.SystemAnalysis.get_frequency_response(g4.S, freqs))
T_resp = np.array(ctrl.SystemAnalysis.get_frequency_response(g4.T, freqs))
if np.max(np.abs(S_resp + T_resp - 1.0)) > 1e-9:
    print("[FAIL] S + T != I pointwise")
    ok = False

# T(DC) = 0.2/0.6 = 1/3 is the Hinf peak for this monotonic 1st-order loop.
if abs(g4n.norm_T - 1.0 / 3.0) > 1e-3:
    print("[FAIL] norm_T should equal T(DC)=1/3")
    ok = False

# S peaks above 1 away from DC (waterbed effect) for this loop - not a bug.
if g4n.norm_S <= 1.0:
    print("[FAIL] norm_S should exceed 1 for this loop")
    ok = False

if abs(dm.alpha - 1.0 / g4n.norm_S) > 1e-6:
    print("[FAIL] disk margin alpha should equal 1/||S||_inf")
    ok = False

print("[PASS] All checks passed." if ok else "[FAIL] One or more checks failed.")
