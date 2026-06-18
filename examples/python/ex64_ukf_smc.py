"""
ex64_ukf_smc.py
OBSERVER + STATE FEEDBACK: UKF + SMC on nonlinear pendulum.

UKF provides filtered state estimates; SMC uses them for robust sliding control.
"""

import _setup_bindings  # noqa: F401
import ctrl_toolbox as ctrl
import numpy as np


def main():
    Ts = 0.02
    g, L, b = 9.81, 1.0, 0.5

    def f_ukf(x, u):
        xn = np.zeros(2)
        xn[0] = x[0] + Ts * x[1]
        xn[1] = x[1] + Ts * (-g/L * np.sin(x[0]) - b * x[1] + u[0])
        return xn

    def h_ukf(x, u):
        return x[:1]

    Q_ukf = np.diag([1e-4, 1e-3])
    R_ukf = np.array([[0.01]])
    # alpha=1.0 with kappa=0, n=2: lambda=0 -> Wc0=2>0, avoids negative covariance weight
    ukf = ctrl.UnscentedKalmanFilter(2, 1, f_ukf, h_ukf, Q_ukf, R_ukf, Ts,
                                     alpha=1.0, beta=2.0, kappa=0.0)

    # SMC parameters
    lambda_smc = 5.0
    eta_smc    = 3.0
    phi_smc    = 0.2

    np.random.seed(42)
    x_true = np.array([0.5, 0.0])
    sse_theta = 0.0
    N = 400

    for k in range(N):
        ym = np.array([x_true[0] + 0.1 * np.random.randn()])
        uv = np.zeros(1)
        ukf.predict(uv)
        ukf.update(ym, uv)
        x_est = ukf.state()

        s = x_est[1] + lambda_smc * x_est[0]
        sign_s = 1.0 if s > 0 else -1.0
        u_smc = -(eta_smc + abs(g/L * np.sin(x_est[0]))) * (s/phi_smc if abs(s) < phi_smc else sign_s)
        uv = np.array([u_smc])

        ddth = -g/L * np.sin(x_true[0]) - b*x_true[1] + u_smc + 0.1*np.random.randn()*0.1
        x_true[0] += Ts * x_true[1]
        x_true[1] += Ts * ddth
        sse_theta += x_true[0]**2

    rmse = float(np.sqrt(sse_theta / N))
    print(f"UKF+SMC pendulum: RMSE theta = {rmse:.4f} rad")
    assert rmse < 0.8 and np.isfinite(rmse), f"RMSE too large: {rmse:.4f}"
    print("PASS")


if __name__ == "__main__":
    main()
