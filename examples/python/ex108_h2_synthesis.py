"""
ex108_h2_synthesis.py

Phase 4 (Iteration 3): discrete H2/LQG output-feedback synthesis.

Builds a small D11=0 generalised plant by hand (MixedSensitivity-built plants have
D11 != 0 from their W1/W3 weight gains - see lib/DiscreteH2.h) and synthesises an
H2-optimal controller for it via ctrl.DiscreteH2.solve().
"""

import sys
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'DiscreteH2'):
        raise AttributeError("DiscreteH2 not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

# nw=2 deliberately - a single noise channel (nw=1) is a degenerate case for the filter
# Riccati: S2^2 == Q2*R2 always holds for a scalar B1/D21, forcing Y=0 and an overly
# aggressive (destabilising) observer gain - see lib/DiscreteH2.cpp's solve() comment.
P = ctrl.GeneralisedPlant()
P.Ts = 0.1
P.A   = np.array([[0.9]])
P.B1  = np.array([[0.3, 0.1]])   # process noise input
P.B2  = np.array([[1.0]])        # control input
P.C1  = np.array([[1.0], [0.3]])  # state + cross-term cost
P.C2  = np.array([[1.0]])        # measurement = state
P.D11 = np.zeros((2, 2))
P.D12 = np.array([[0.2], [1.0]])  # control-cost row
P.D21 = np.array([[0.1, 0.4]])   # measurement noise
P.D22 = np.zeros((1, 1))

result = ctrl.DiscreteH2.solve(P)
print(f"H2: feasible={result.feasible}  achieved_h2_norm={result.achieved_h2_norm:.4f}")

ok = result.feasible and np.isfinite(result.achieved_h2_norm) and result.achieved_h2_norm > 0.0

if result.feasible:
    h2 = ctrl.DiscreteH2(result)
    u = h2.compute(0.1)
    print(f"DiscreteH2 compute(0.1) = {u:.4f}")
    ok = ok and np.isfinite(u)

print("PASS" if ok else "FAIL")
sys.exit(0 if ok else 1)
