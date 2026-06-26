"""
ex130_gp_mpc.py

Phase 3 (ML3): GP-uncertainty-aware tightening of NonlinearMPC's input bounds.

Mirrors ex113_gp_mpc.cpp -- compares plain NonlinearMPC against GPMPC on the same scalar
plant: unfitted GP behaves identically (regression); a GP trained far from the operating
point (high posterior variance there) visibly tightens GPMPC's input bounds.
"""

import sys
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'GPMPC'):
        raise AttributeError("GPMPC not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)


def f_dyn(x, u):
    return np.array([0.9 * x[0] + u[0]])


params = ctrl.GPMPCParams()
params.nmpc.Np = 5
params.nmpc.Nu = 3
params.nmpc.n_states = 1
params.nmpc.n_inputs = 1
params.nmpc.n_outputs = 1
params.nmpc.uMin = -5.0
params.nmpc.uMax = 5.0
params.nmpc.Ts = 0.1

# Real binding names confirmed in bindings/controllers_bindings.cpp:1860,1886 --
# ctrl.GPParams (NOT GaussianProcessParams) and ctrl.GPResidualParams (NOT GPResidualModelParams).
gp_p = ctrl.GPParams()
gp_p.length_scale = 0.5
gp_p.signal_var = 1.0
gp_p.noise_var = 0.01

print("=== GPMPC: unfitted GP (regression check) ===")
resid_p = ctrl.GPResidualParams()
resid_p.gp = gp_p
gp_unfitted = ctrl.GPResidualModel(2, resid_p)
nmpc = ctrl.NonlinearMPC(params.nmpc, f_dyn)
gpmpc = ctrl.GPMPC(params, f_dyn, gp_unfitted)

x = np.array([1.0])
ok = True
for _ in range(5):
    nmpc.set_state(x)
    gpmpc.set_state(x)
    u_nmpc = nmpc.compute(0.5 - x[0])
    u_gpmpc = gpmpc.compute(0.5 - x[0])
    ok = ok and abs(u_nmpc - u_gpmpc) < 1e-9
    x = f_dyn(x, np.array([u_nmpc]))

print(f"NonlinearMPC == GPMPC(unfitted): {'yes' if ok else 'NO (bug)'}")
print(f"max tightening: {gpmpc.last_tightening().max():.6f} (expect 0)\n")
if not ok:
    sys.exit(1)

print("=== GPMPC: GP trained far from x=1.0 (high local variance) ===")
resid_p2 = ctrl.GPResidualParams()
resid_p2.gp = gp_p
gp_far = ctrl.GPResidualModel(2, resid_p2)
gp_far.add_residual_point(np.array([50.0, 50.0]), 0.0, 0.0)
gp_far.fit()
gpmpc2 = ctrl.GPMPC(params, f_dyn, gp_far)

x2 = np.array([1.0])
gpmpc2.set_state(x2)
u2 = gpmpc2.compute(0.5 - x2[0])
print(f"u = {u2:.5f}  max tightening = {gpmpc2.last_tightening().max():.6f}")

ok2 = np.isfinite(u2) and gpmpc2.last_tightening().max() > 0.0
print("\n[PASS] All checks passed." if ok2 else "\n[FAIL] One or more checks failed.")
sys.exit(0 if ok2 else 1)
