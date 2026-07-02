"""
ex142_lqr_adapter.py

LQRAdapter - the IController shim over the stateless DiscreteLQR.

DiscreteLQR is pure math (gain via DARE); LQRAdapter wires it to state/reference
callbacks so it can be used through the IController interface (and inside a
ControllerStack). This regulates a lightly-damped 2nd-order plant from a nonzero
initial state back to the origin using state-feedback via the adapter callbacks.
"""

import sys
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'LQRAdapter'):
        raise AttributeError("LQRAdapter not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

Ts = 0.1
Ac = np.array([[0.0, 1.0], [-1.0, -0.6]])
Bc = np.array([[0.0], [1.0]])
Cc = np.array([[1.0, 0.0]])
Dc = np.zeros((1, 1))
plant = ctrl.c2d(ctrl.StateSpace(Ac, Bc, Cc, Dc, 0.0), Ts, ctrl.C2dMethod.ZOH)

lqr_p = ctrl.LQRParams()
lqr_p.Q = np.diag([10.0, 1.0])
lqr_p.R = np.array([[0.1]])
lqr = ctrl.DiscreteLQR(plant, lqr_p)
assert lqr.dare_converged(), "DARE did not converge"

# state_fn reads the live state; ref_fn omitted -> regulate to the origin.
x = np.array([1.0, 0.0])          # nonzero initial condition
adapter = ctrl.LQRAdapter(lqr, state_fn=lambda: x)

print("=== LQRAdapter state-feedback regulation ===")
print(f"  K = {lqr.gain_matrix()}")
for k in range(80):
    u = adapter.compute_vec()      # full MIMO control vector from callbacks
    y_vec, x = ctrl.ss_step_copy(plant, x, u)
    if k % 10 == 0:
        print(f"k={k:3d}  x=[{x[0]:+.4f}, {x[1]:+.4f}]  u={float(u[0]):+.4f}")

xn = float(np.linalg.norm(x))
print(f"final |x| = {xn:.5f}")

ok = adapter.is_healthy() and np.all(np.isfinite(x)) and xn < 0.05
print("[PASS]" if ok else "[FAIL]")
sys.exit(0 if ok else 1)
