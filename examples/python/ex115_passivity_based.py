"""
ex115_passivity_based.py

Phase 3 (NC2): Passivity-based (PD+) regulation of a single pendulum.

Mirrors ex98_passivity_based.cpp - regulates a single-pendulum Euler-Lagrange system to a
nonzero desired angle, monitoring the shaped storage-energy function's non-increase.
"""

import sys
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'PassivityBasedController'):
        raise AttributeError("PassivityBasedController not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

Ts, ml2, mgl = 0.01, 1.0, 9.8
params = ctrl.PBCParams()
params.Kp = np.array([[10.0]])
params.Kd = np.array([[4.0]])

pbc = ctrl.PassivityBasedController(
    lambda q: np.array([[ml2]]),
    lambda q: np.array([mgl * np.sin(q[0])]),
    lambda q, qdot: np.array([[0.0]]),
    params, Ts)
pbc.set_desired(np.array([0.5]))

state = np.zeros(2)  # [q, qdot]
prev_energy = np.inf
ever_increased = False
for k in range(3000):
    u = pbc.compute_vec(state)
    energy = pbc.storage_energy()
    if k > 50 and energy > prev_energy + 1e-6:
        ever_increased = True
    prev_energy = energy

    qddot = (u[0] - mgl * np.sin(state[0])) / ml2
    state[1] += Ts * qddot
    state[0] += Ts * state[1]

print(f"Final angle: q={state[0]:.4f} (desired=0.5)  storage energy={pbc.storage_energy():.6f}")

ok = np.all(np.isfinite(state)) and abs(state[0] - 0.5) < 0.02 and not ever_increased
print("[PASS] All checks passed." if ok else "[FAIL] One or more checks failed.")
sys.exit(0 if ok else 1)
