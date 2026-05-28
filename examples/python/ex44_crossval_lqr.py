"""
ex44 - Cross-Validation: ctrl.DiscreteLQR vs scipy DARE / control.dlqr
=======================================================================
Goal     : Verify that DiscreteLQR solves the DARE and computes the optimal
           feedback gain K* to the same precision as scipy and python-control.

Agreement criterion:
  - |K_ctrl - K_scipy| < 1e-6 (relative) for each element
  - Closed-loop eigenvalues |lambda_ctrl - lambda_scipy| < 1e-8

Plants tested:
  1. Double integrator  (Ts=0.01, Q=10*I2, R=I1)
  2. DC motor (Ts=0.05, Q=diag(100,1), R=[[0.01]])

Run:
    conda run -n soft_robotics -- python ex44_crossval_lqr.py
"""
import _setup_bindings  # noqa: F401
import ctrl_toolbox as ctrl
import numpy as np
from scipy import linalg
import control

TOL_REL = 1e-6
results = []

def check_K(label, K_ctrl, K_ref, tol=TOL_REL):
    ok = np.allclose(K_ctrl, K_ref, rtol=tol, atol=1e-10)
    err = np.max(np.abs(K_ctrl - K_ref))
    print(f"  {label:45s}: {'PASS' if ok else f'FAIL (max_err={err:.2e})'}")
    return ok

# ===========================================================================
# Plant 1: double integrator
# A = [[1, Ts], [0, 1]],  B = [[Ts^2/2], [Ts]],  Ts=0.01
# Q = 10*I2,  R = I1
# ===========================================================================
print("=== LQR cross-validation: double integrator ===")
Ts = 0.01
Ad_int = np.array([[1.0, Ts], [0.0, 1.0]])
Bd_int = np.array([[Ts**2/2], [Ts]])
Cd_int = np.array([[1.0, 0.0]])
Dd_int = np.zeros((1, 1))
plant_int = ctrl.StateSpace(Ad_int, Bd_int, Cd_int, Dd_int, Ts)

Q_int = 10.0 * np.eye(2)
R_int = np.eye(1)
lqr_p = ctrl.LQRParams()
lqr_p.Q = Q_int; lqr_p.R = R_int
lqr = ctrl.DiscreteLQR(plant_int, lqr_p)

# scipy reference: solve_discrete_are directly
P_scipy = linalg.solve_discrete_are(Ad_int, Bd_int, Q_int, R_int)
K_scipy = np.linalg.solve(R_int + Bd_int.T @ P_scipy @ Bd_int, Bd_int.T @ P_scipy @ Ad_int)

# python-control reference
K_ctrl_lib, _, _ = control.dlqr(Ad_int, Bd_int, Q_int, R_int)

K_ctrl = lqr.gain_matrix()
print(f"  K_ctrl  = {K_ctrl}")
print(f"  K_scipy = {K_scipy}")
print(f"  K_ctrl_lib = {K_ctrl_lib}")

results.append(check_K("K vs scipy.linalg.solve_discrete_are", K_ctrl, K_scipy))
results.append(check_K("K vs control.dlqr",                    K_ctrl, K_ctrl_lib))

# Riccati solution P should match
P_ctrl = lqr.riccati_solution()
ok_P = np.allclose(P_ctrl, P_scipy, rtol=TOL_REL, atol=1e-10)
print(f"  P vs scipy DARE: {'PASS' if ok_P else 'FAIL'}")
results.append(ok_P)

# Closed-loop eigenvalues (similarity-invariant sanity check)
A_cl_ctrl  = Ad_int - Bd_int @ K_ctrl
A_cl_scipy = Ad_int - Bd_int @ K_scipy
eig_ctrl  = np.sort(np.abs(np.linalg.eigvals(A_cl_ctrl)))
eig_scipy = np.sort(np.abs(np.linalg.eigvals(A_cl_scipy)))
ok_eig = np.allclose(eig_ctrl, eig_scipy, atol=1e-8)
print(f"  Closed-loop |eigs|: ctrl={eig_ctrl}  scipy={eig_scipy}  match={ok_eig}")
results.append(ok_eig)

# Verify both closed-loop systems are stable (all eigs inside unit disk)
assert all(eig_ctrl < 1.0), "LQR closed-loop not stable"
assert lqr.dare_converged(), "DARE did not converge"

# ===========================================================================
# Plant 2: DC motor (J=0.01, b=0.1, Km=0.01, Ra=1.0), Ts=0.05
# ===========================================================================
print("\n=== LQR cross-validation: DC motor ===")
J=0.01; b=0.1; Km=0.01; Ra=1.0; Ts2=0.05
sys_dc_c = ctrl.StateSpace(
    np.array([[0.0, 1.0], [0.0, -b/J]]),
    np.array([[0.0], [Km/(J*Ra)]]),
    np.array([[1.0, 0.0]]), np.zeros((1,1)), 0.0)
plant_dc = ctrl.c2d(sys_dc_c, Ts2, ctrl.C2dMethod.ZOH)

Q_dc = np.diag([100.0, 1.0])
R_dc = np.array([[0.01]])
lqr_p2 = ctrl.LQRParams()
lqr_p2.Q = Q_dc; lqr_p2.R = R_dc
lqr_dc = ctrl.DiscreteLQR(plant_dc, lqr_p2)

Ad_dc = plant_dc.A; Bd_dc = plant_dc.B
P_scipy2 = linalg.solve_discrete_are(Ad_dc, Bd_dc, Q_dc, R_dc)
K_scipy2 = np.linalg.solve(R_dc + Bd_dc.T @ P_scipy2 @ Bd_dc, Bd_dc.T @ P_scipy2 @ Ad_dc)
K_ctrl2, _, _ = control.dlqr(Ad_dc, Bd_dc, Q_dc, R_dc)

results.append(check_K("K vs scipy.linalg.solve_discrete_are", lqr_dc.gain_matrix(), K_scipy2))
results.append(check_K("K vs control.dlqr",                    lqr_dc.gain_matrix(), K_ctrl2))

n_pass = sum(results)
n_total = len(results)
print(f"\n{'='*50}")
print(f"LQR cross-validation: {n_pass}/{n_total} checks passed")
assert n_pass == n_total
print("PASS")
