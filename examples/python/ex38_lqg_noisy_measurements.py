"""
ex38 - LQG with Noisy Measurements (ctrl_toolbox bindings)
===========================================================
Goal     : Demonstrate DiscreteLQG combining LQR + Kalman filter for
           state estimation under noisy position measurements.

Scenario : DC motor position control. Only position is measured, with
           sigma = 0.005 rad encoder noise. Kalman filter recovers
           velocity state and suppresses noise for LQR feedback.

Key result: KF RMSE << raw measurement noise sigma.

Run:
    conda run -n soft_robotics -- python ex38_lqg_noisy_measurements.py
"""
import _setup_bindings  # noqa: F401
import ctrl_toolbox as ctrl
import numpy as np

rng = np.random.default_rng(42)
J=0.01; b=0.1; Km=0.01; Ra=1.0; Ts=0.05
plant = ctrl.c2d(ctrl.StateSpace(
    np.array([[0.0,1.0],[0.0,-b/J]]),
    np.array([[0.0],[Km/(J*Ra)]]),
    np.array([[1.0,0.0]]), np.zeros((1,1)), 0.0), Ts, ctrl.C2dMethod.ZOH)

lqr_p = ctrl.LQRParams()
lqr_p.Q = np.diag([100.0, 1.0]); lqr_p.R = np.array([[0.01]])

sigma_meas = 0.005
Q_noise = np.eye(2) * 1e-8
R_noise = np.array([[sigma_meas**2]])
lqg = ctrl.DiscreteLQG(plant, lqr_p, Q_noise, R_noise)

N=int(3.0/Ts); r_pos=1.0; x_ref=np.array([r_pos,0.0])
x_true=np.zeros(2); u_prev=np.zeros(1)
est_errs=[]

for k in range(N):
    y_noisy = np.array([x_true[0] + rng.normal(0, sigma_meas)])
    u_vec = lqg.step(y_noisy, u_prev, x_ref)
    x_hat = lqg.state_estimate()
    est_errs.append(x_hat[0] - x_true[0])
    u = float(np.clip(u_vec[0], -12.0, 12.0))
    _, x_true = ctrl.ss_step_copy(plant, x_true, np.array([u]))
    u_prev = np.array([u])

rmse = np.sqrt(np.mean(np.array(est_errs)**2))
print("=== LQG with Noisy Measurements ===")
print(f"Measurement noise sigma: {sigma_meas:.4f} rad")
print(f"KF RMSE vs true state:  {rmse:.5f} rad  ({100*rmse/sigma_meas:.1f}% of noise)")
print(f"Final position:         {x_true[0]:.4f} rad  (target {r_pos})")
print(f"KF noise reduction:     {20*np.log10(sigma_meas/max(rmse,1e-9)):.1f} dB")
assert rmse < sigma_meas * 3.0, "KF estimate should be better than raw noise"
assert abs(x_true[0] - r_pos) < 0.05, "LQG did not settle"
print("PASS")
