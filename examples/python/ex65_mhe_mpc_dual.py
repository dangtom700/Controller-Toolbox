"""
ex65_mhe_mpc_dual.py
OBSERVER + STATE FEEDBACK: MHE (state estimator) + MPC (optimal control).

The dual pairing: MPC optimises future inputs, MHE optimises past state estimates.
Together they form the optimal output-feedback controller for constrained systems.
"""

import _setup_bindings  # noqa: F401
import ctrl_toolbox as ctrl
import numpy as np


def main():
    Ts = 0.2

    A = np.array([[1.0, Ts], [0.0, 0.97]])
    B = np.array([[0.5*Ts*Ts], [Ts]])
    C = np.array([[1.0, 0.0]])
    D = np.zeros((1, 1))
    plant = ctrl.StateSpace(A, B, C, D, Ts)

    # MHE
    Q_mhe = 1e-3 * np.eye(2)
    R_mhe = 0.1 * np.eye(1)
    mhe_p = ctrl.MHEParams()
    mhe_p.N    = 8
    mhe_p.wMin = -0.5
    mhe_p.wMax =  0.5
    mhe = ctrl.MovingHorizonEstimator(plant, Q_mhe, R_mhe, mhe_p)
    mhe.initialize(np.zeros(2), 10.0 * np.eye(2))

    # MPC
    mp = ctrl.MPCParams()
    mp.Np = 12; mp.Nc = 4; mp.rho_y = 5.0; mp.rho_u = 0.2
    mp.uMin = -3.0; mp.uMax = 3.0
    mpc = ctrl.DiscreteMPC(plant, mp)

    np.random.seed(42)
    x_true = np.zeros(2)
    ref = 1.0
    N = 150
    final_y = 0.0

    u_prev = np.zeros(1)
    for k in range(N):
        ym = np.array([x_true[0] + 0.2 * np.random.randn()])
        x_est = mhe.estimate(ym, u_prev)

        mpc.set_state(x_est)  # inject MHE estimate as MPC initial condition
        u_mpc = mpc.compute(ref - float(ym[0]))
        u_prev = np.array([u_mpc])

        w = 0.05 * np.random.randn(2)
        x_true = A @ x_true + B.flatten() * u_mpc + w * 0.05
        final_y = x_true[0]

    print(f"MHE+MPC: final pos = {final_y:.4f}  (ref={ref})")
    print(f"MHE converged: {mhe.last_converged()}  MPC converged: {mpc.last_qp_converged()}")
    assert np.isfinite(final_y) and abs(final_y - ref) < 0.4, \
        f"Did not converge: y={final_y:.4f}"
    print("PASS")


if __name__ == "__main__":
    main()
