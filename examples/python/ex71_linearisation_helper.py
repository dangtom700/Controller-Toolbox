"""
ex71_linearisation_helper.py
Demonstrates ctrl.jacobian_x / jacobian_u and ctrl.linearise_at_point via the
pybind11 binding.

Van der Pol oscillator (mu=0.5) at x=(0,0), u=0:

  Analytical Jacobian:
    A_c = [[0, 1], [-1, 0.5]]
    B_c = [[0], [1]]

  Test:
    1. Numerical A_c / B_c match analytical to within 1e-3 relative error.
    2. linearise_at_point returns a discrete StateSpace with A rows=cols=2.
    3. DiscreteLQR designed on the linearised model drives the LINEARISED
       simulation to ||x|| < 0.05 after 500 steps from x=(0.5, 0).
"""

import _setup_bindings  # noqa: F401
import ctrl_toolbox as ctrl
import numpy as np


def main():
    mu = 0.5
    Ts = 0.05

    # Van der Pol continuous-time dynamics
    def vdp(x, u):
        xd = np.zeros(2)
        xd[0] = x[1]
        xd[1] = mu * (1.0 - x[0]**2) * x[1] - x[0] + u[0]
        return xd

    x0 = np.zeros(2)
    u0 = np.zeros(1)

    # --- Numerical Jacobians ---
    A_num = ctrl.jacobian_x(vdp, x0, u0)
    B_num = ctrl.jacobian_u(vdp, x0, u0)

    A_ana = np.array([[0.0, 1.0], [-1.0, mu]])
    B_ana = np.array([[0.0], [1.0]])

    A_err = np.linalg.norm(A_num - A_ana) / (np.linalg.norm(A_ana) + 1e-12)
    B_err = np.linalg.norm(B_num - B_ana) / (np.linalg.norm(B_ana) + 1e-12)

    print(f"Jacobian relative errors:  A={A_err:.2e}  B={B_err:.2e}")
    assert A_err < 1e-3 and B_err < 1e-3, \
        f"Jacobian error too large: A={A_err:.2e}, B={B_err:.2e}"

    # --- linearise_at_point ---
    sys_d = ctrl.linearise_at_point(vdp, x0, u0, Ts)
    assert sys_d.state_size() == 2, "Expected 2-state system"
    assert sys_d.input_size() == 1, "Expected 1-input system"
    print(f"Discrete A:\n{sys_d.A}\nDiscrete B:\n{sys_d.B}")

    # --- LQR on linearised model -> close loop on linearised plant ---
    lp = ctrl.LQRParams()
    lp.Q = np.diag([10.0, 10.0])
    lp.R = np.eye(1)
    lqr = ctrl.DiscreteLQR(sys_d, lp)
    assert lqr.dare_converged(), "DARE did not converge"

    K = lqr.gain_matrix()   # shape (1, 2)
    x = np.array([0.5, 0.0])
    A_d = sys_d.A
    B_d = sys_d.B           # shape (2, 1)

    for _ in range(500):
        u = -(K @ x)        # shape (1,)
        x = A_d @ x + (B_d @ u.reshape(1, 1)).flatten()

    norm_final = float(np.linalg.norm(x))
    print(f"LQR on linearised Van der Pol: ||x(T)|| = {norm_final:.4f}  (need < 0.05)")
    assert norm_final < 0.05, f"Did not converge: ||x||={norm_final:.4f}"
    print("PASS")


if __name__ == "__main__":
    main()
