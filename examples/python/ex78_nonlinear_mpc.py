"""
ex78_nonlinear_mpc.py
---------------------
Part 22: NonlinearMPC binding demonstration.

Plant (discrete, Ts=0.1s):
  x1[k+1] = 0.9*x1 - 0.05*x1^3 + x2*0.1
  x2[k+1] = 0.8*x2 + u
  y = x1

NMPC minimises ||y - 1||^2 over Np=10 steps.
Acceptance: |y_final - 1| < 0.1 and QP converged.

Expected output: PASS
"""
import sys
import numpy as np

sys.path.insert(0, __file__.rsplit("\\", 1)[0])
import _setup_bindings
try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'NonlinearMPC'):
        raise AttributeError("NonlinearMPC not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

Ts = 0.1
N_steps = 80

# Discrete-time nonlinear dynamics
def f_nl(x, u):
    xn = np.empty(2)
    xn[0] = 0.9 * x[0] - 0.05 * x[0]**3 + 0.1 * x[1]
    xn[1] = 0.8 * x[1] + float(u[0])
    return xn

C_out = np.array([[1.0, 0.0]])  # y = x1

# NMPC parameters
np_p = ctrl.NMPCParams()
np_p.Np = 10; np_p.Nu = 3
np_p.rho_y = 5.0; np_p.rho_u = 0.5
np_p.uMin = -5.0; np_p.uMax = 5.0
np_p.qpMaxIter = 500; np_p.qpTol = 1e-6
np_p.Ts = Ts
np_p.n_states = 2; np_p.n_inputs = 1; np_p.n_outputs = 1

nmpc = ctrl.NonlinearMPC(np_p, f_nl, C_out)

x = np.zeros(2)
y_ref = np.array([1.0])

for k in range(N_steps):
    y = float((C_out @ x)[0])
    nmpc.set_state(x)
    u_vec = nmpc.compute_ref(x, y_ref)
    x = f_nl(x, u_vec)

y_final = float((C_out @ x)[0])
converged = nmpc.last_qp_converged()

print(f"Final y = {y_final:.4f}  |y-ref| = {abs(y_final - 1.0):.4f}  QP converged = {converged}")

if abs(y_final - 1.0) < 0.15 and converged:
    print("PASS")
else:
    print(f"FAIL: y_final={y_final:.4f}, converged={converged}")
    sys.exit(1)
