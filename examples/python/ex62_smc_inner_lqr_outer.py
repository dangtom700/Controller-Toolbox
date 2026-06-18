"""
ex62_smc_inner_lqr_outer.py
CASCADE: SMC (inner) + LQR (outer).

SMC corrects velocity-level disturbances; LQR nominal optimal control for position.
"""

import _setup_bindings  # noqa: F401
import ctrl_toolbox as ctrl
import numpy as np


def main():
    Ts = 0.01

    # SMC for inner velocity tracking
    smc_p = ctrl.SMCParams()
    smc_p.c_e  = 1.0             # error weight in sliding surface
    smc_p.c_de = 10.0 * Ts      # c_de = lambda * Ts (lambda=10 [1/s])
    smc_p.K    = 5.0             # switching gain
    smc_p.phi  = 0.05
    smc_p.uMin = -20.0
    smc_p.uMax =  20.0
    smc = ctrl.DiscreteSMC(smc_p, Ts)

    # LQR for outer position tracking (double integrator)
    A = np.array([[1.0, Ts], [0.0, 1.0]])
    B = np.array([[0.5*Ts*Ts], [Ts]])
    C = np.array([[1.0, 0.0]])
    D = np.zeros((1, 1))
    plant = ctrl.StateSpace(A, B, C, D, Ts)

    lqr_p = ctrl.LQRParams()
    lqr_p.Q = np.diag([10.0, 1.0])
    lqr_p.R = np.eye(1) * 0.1
    lqr = ctrl.DiscreteLQR(plant, lqr_p)
    K = lqr.gain_matrix()

    vel_sys_c = ctrl.StateSpace(
        np.array([[-5.0]]),
        np.array([[5.0]]),
        np.array([[1.0]]),
        np.zeros((1, 1)), 0.0
    )
    vel_sys = ctrl.c2d(vel_sys_c, Ts, ctrl.C2dMethod.ZOH)
    x_vel = np.zeros(vel_sys.state_size())

    pos, vel = 0.0, 0.0
    ref_pos = 1.0
    N = 1000

    for k in range(N):
        x_err = np.array([ref_pos - pos, 0.0 - vel])
        vel_ref = float(np.squeeze(K @ x_err))

        e_smc = vel - vel_ref
        u_smc = smc.compute(e_smc)

        y_vel_vec, x_vel = ctrl.ss_step_copy(vel_sys, x_vel, np.array([u_smc]))
        vel = float(np.squeeze(y_vel_vec))
        pos += Ts * vel

    print(f"CASCADE SMC(inner)+LQR(outer): final pos={pos:.4f}  (ref={ref_pos})")
    assert abs(pos - ref_pos) < 0.05, f"Did not converge: pos={pos:.4f}"
    print("PASS")


if __name__ == "__main__":
    main()
