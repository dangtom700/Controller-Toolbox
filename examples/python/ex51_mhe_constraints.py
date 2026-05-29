"""
ex51_mhe_constraints.py
MovingHorizonEstimator Python binding demo with process-noise box constraints.

Uses the ctrl_toolbox MHE binding to estimate a 2-state system and verifies
that the estimation error stays bounded over 80 steps.
"""

import _setup_bindings  # noqa: F401
import ctrl_toolbox as ctrl
import numpy as np

def main():
    Ts = 0.5

    # 2-state heating system: T1, T2
    A = np.array([[0.90, 0.05],
                  [0.08, 0.88]])
    B = np.array([[0.1], [0.05]])
    C = np.array([[1.0, 0.0]])
    D = np.zeros((1, 1))

    plant = ctrl.StateSpace(A, B, C, D, Ts)

    Q_proc = 0.01 * np.eye(2)
    R_meas = 0.05 * np.eye(1)

    params = ctrl.MHEParams()
    params.N        = 8
    params.wMin     = -0.5
    params.wMax     =  0.5
    params.qpMaxIter = 200

    mhe = ctrl.MovingHorizonEstimator(plant, Q_proc, R_meas, params)
    mhe.initialize(np.zeros(2), 10.0 * np.eye(2))

    x_true = np.zeros(2)
    np.random.seed(7)

    sse = 0.0
    for k in range(80):
        u_k = 1.0 if k < 40 else 0.0
        u = np.array([u_k])

        # True plant update with small noise
        w = 0.02 * (np.random.rand(2) - 0.5)
        x_true = A @ x_true + B @ u + w

        # Noisy measurement (C*x + v)
        v = 0.05 * (np.random.rand(1) - 0.5)
        y = C @ x_true + v

        x_est = mhe.estimate(y, u)
        err = float(np.squeeze(x_est[0])) - float(x_true[0])
        sse += err ** 2

    rmse = float(np.sqrt(sse / 80))
    print(f"MHE RMSE (x1) over 80 steps: {rmse:.4f}")
    print(f"MHE last converged: {mhe.last_converged()}")
    assert rmse < 2.0,      f"RMSE too large: {rmse:.4f}"
    assert mhe.last_converged(), "MHE QP did not converge"
    print("PASS")


if __name__ == "__main__":
    main()
