"""
ex136_lp_mpc.py

LPMPC (Phase 4 OC4) - SISO L1-cost linear MPC closed-loop step tracking.

Mirrors ex119_lp_mpc.cpp: same plant as ex01_tf_pid (G(s) = 1/(s^2+1.5s+1), ZOH at Ts=0.01s).
LPMPC casts L1 tracking + L1 move-suppression as an LP each step (LPSolver two-phase simplex)
instead of DiscreteMPC's L2/QP.
"""

import sys
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'LPMPC'):
        raise AttributeError("LPMPC not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

Ts = 0.01
plant_tf = ctrl.TransferFunction([0.0, 4.9625e-5, 4.9125e-5],
                                  [1.0, -1.98511, 0.98522], Ts)
plant = ctrl.tf2ss(plant_tf)

params = ctrl.LPMPCParams()
params.Np, params.Nc = 15, 5
params.rho_y, params.rho_u = 1.0, 0.001  # see ex119's comment: L1-MPC deadzone, keep rho_u low
params.uMin, params.uMax = -5.0, 5.0
params.duMin, params.duMax = -1.0, 1.0

mpc = ctrl.LPMPC(plant, params)

x = np.zeros(plant.state_size())
y = 0.0
ref = 1.0
max_abs_u = 0.0
all_converged = True

for k in range(1500):
    e = ref - y
    u = mpc.compute(e)
    all_converged = all_converged and mpc.last_lp_converged()
    max_abs_u = max(max_abs_u, abs(u))

    y_vec, x = ctrl.ss_step_copy(plant, x, np.array([u]))
    y = float(np.squeeze(y_vec))

    if k % 150 == 0:
        print(f"k={k:4d}  y={y:.4f}  e={e:.4f}  u={u:.4f}")

print(f"Final y={y:.4f}, max|u|={max_abs_u:.4f}, all LP solves converged={all_converged}")

ok = all_converged and abs(y - ref) < 0.02 and max_abs_u <= 5.0 + 1e-9
print("[PASS]" if ok else "[FAIL]")
sys.exit(0 if ok else 1)
