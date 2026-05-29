"""
ex60_benchmark_distillation.py
MIMO distillation column control using DiscreteLQG (Wood-Berry plant).

Wood-Berry distillation column: 2-input 2-output system with transport delays.
Uses the approximate delay-free model for LQG design, then validates closed-loop.

Reference: Wood & Berry, AIChE Journal (1973).
  G(s) = [12.8*exp(-s)/(16.7s+1),  -18.9*exp(-3s)/(21s+1) ]
          [ 6.6*exp(-7s)/(10.4s+1), -19.4*exp(-3s)/(14.4s+1)]

For the LQG demo we use the delay-free (theta=0) model to keep the state-space
tractable, then note that a full design would use Smith predictors on each channel.
"""

import _setup_bindings  # noqa: F401
import ctrl_toolbox as ctrl
import numpy as np
from scipy.signal import cont2discrete


def make_wood_berry_ss(Ts):
    """Delay-free 2x2 Wood-Berry plant in state-space (ZOH)."""
    # Four first-order TF channels: each becomes a 1-state system
    # G11 = 12.8/(16.7s+1), G12 = -18.9/(21s+1)
    # G21 = 6.6/(10.4s+1), G22 = -19.4/(14.4s+1)
    taus = [16.7, 21.0, 10.4, 14.4]
    gains = [12.8, -18.9, 6.6, -19.4]
    n = 4

    Ac = -np.diag([1.0/tau for tau in taus])  # 4x4 diagonal
    Bc = np.zeros((4, 2))
    Bc[0, 0] = gains[0] / taus[0]   # G11 state driven by u1
    Bc[1, 1] = gains[1] / taus[1]   # G12 state driven by u2
    Bc[2, 0] = gains[2] / taus[2]   # G21 state driven by u1
    Bc[3, 1] = gains[3] / taus[3]   # G22 state driven by u2

    Cc = np.array([[1, 0, 0, 0],   # y1 = x_G11 + x_G12  (channels sum to output)
                   [0, 0, 1, 0]])   # y2 = x_G21 + x_G22
    Cc = np.array([[1, 1, 0, 0],
                   [0, 0, 1, 1]])

    Dc = np.zeros((2, 2))

    Ad, Bd, Cd, Dd, _ = cont2discrete((Ac, Bc, Cc, Dc), Ts, method='zoh')
    plant = ctrl.StateSpace(Ad, Bd, Cd, Dd, Ts)
    return plant


def main():
    Ts  = 0.5   # 0.5 min sample time (Wood-Berry uses minutes)
    plant = make_wood_berry_ss(Ts)

    n, m, p = plant.state_size(), plant.input_size(), plant.output_size()
    print(f"Wood-Berry SS: n={n}, m={m}, p={p}")

    # LQG design: Q_noise, R_noise for Kalman filter
    Q_noise = 0.1 * np.eye(n)
    R_noise = 0.1 * np.eye(p)

    lqr_p = ctrl.LQRParams()
    lqr_p.Q = 10.0 * np.eye(n)
    lqr_p.R = np.eye(m)

    # Use DiscreteLQR directly (MIMO state-feedback) for clarity.
    # DiscreteLQG binding has SISO compute() only; for MIMO use gain_matrix().
    lqr = ctrl.DiscreteLQR(plant, lqr_p)
    K   = lqr.gain_matrix()
    print(f"LQR gain K shape: {K.shape}")

    # Closed-loop simulation: step setpoint change (regulation to origin from initial state)
    # x_ref = steady-state state for ref output; approximate as x=0 regulation
    x    = np.zeros(n)
    # Start away from origin to test regulation
    x[0] = 1.0; x[1] = -0.5
    N    = 400
    iae  = np.zeros(p)
    final_y = np.zeros(p)

    for k in range(N):
        y = plant.C @ x
        # MIMO state-feedback: u = -K * x
        u = -K @ x
        # Clip control
        u = np.clip(u, -5.0, 5.0)
        x = plant.A @ x + plant.B @ u

        iae += np.abs(y) * Ts
        final_y = y

    print(f"LQR final y1={final_y[0]:.4f}  y2={final_y[1]:.4f} (target=[0,0])")
    print(f"IAE: y1={iae[0]:.4f}  y2={iae[1]:.4f}")

    assert np.all(np.isfinite(final_y)), "Final y not finite"
    assert np.all(np.abs(final_y) < 0.1), \
        f"LQR did not regulate to origin: y={final_y}"
    print("PASS")


if __name__ == "__main__":
    main()
