"""
ex133_dynamic_inversion_feedforward.py

Dynamic-inversion feedforward via invert_transfer_function() + FeedforwardController.

Mirrors ex116_dynamic_inversion_feedforward.cpp -- shows that "dynamic inversion" needs no
dedicated class: invert_transfer_function(G) builds G^-1(z), tf2ss() realises it as a
StateSpace, and the existing FeedforwardController runs it against the reference, giving
perfect open-loop tracking (G^-1(z)*G(z) == 1 identically -- no transient at all).
"""

import sys
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'invert_transfer_function'):
        raise AttributeError("invert_transfer_function not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

Ts = 0.1

# Plant: G(s) = 2/(s+1) + 1 = (s+3)/(s+1) -> ZOH discretisation.
# Must be biproper (D != 0): a strictly-proper plant ZOH-discretises to a strictly-proper
# discrete TF (num[0] == 0), whose causal inverse doesn't exist - invert_transfer_function()
# throws for exactly this case.
plant_c = ctrl.StateSpace(np.array([[-1.0]]), np.array([[1.0]]),
                           np.array([[2.0]]), np.array([[1.0]]), 0.0)
plant = ctrl.c2d(plant_c, Ts, ctrl.C2dMethod.ZOH)
G = ctrl.ss2tf(plant)
print(f"G(z^-1): num={G.num}  den={G.den}")

Ginv = ctrl.invert_transfer_function(G)
print(f"G^-1(z^-1): num={Ginv.num}  den={Ginv.den}")

Gff_model = ctrl.tf2ss(Ginv)
ff = ctrl.FeedforwardController(Gff_model)

x_plant = np.zeros(plant.state_size())
r_seq = [1.0, 1.0, 1.0, 2.5, 2.5, -1.0, -1.0, 0.0, 3.0, 3.0]

ok = True
for r in r_seq:
    u_ff = ff.compute(r)
    y_arr, x_plant = ctrl.ss_step_copy(plant, x_plant, np.array([u_ff]))
    y = float(y_arr[0])
    print(f"  r={r:.4f}  u_ff={u_ff:.6f}  y={y:.6f}")
    ok = ok and np.isfinite(y) and abs(y - r) < 1e-6

print("[PASS]" if ok else "[FAIL]")
sys.exit(0 if ok else 1)
