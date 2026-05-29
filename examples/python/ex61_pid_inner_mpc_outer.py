"""
ex61_pid_inner_mpc_outer.py
CASCADE: PID (inner, fast) + MPC (outer, slow) using ctrl_toolbox binding.

Inner PID corrects fast actuator dynamics; outer MPC provides slow optimal setpoints.
"""

import _setup_bindings  # noqa: F401
import ctrl_toolbox as ctrl
import numpy as np


def main():
    Ts_i = 0.01   # inner loop sample time
    Ts_o = 0.1    # outer loop sample time

    # Inner fast plant G_i(s) = 1/(0.1s+1), ZOH-discretised
    sys_i_c = ctrl.StateSpace(
        np.array([[-10.0]]),
        np.array([[10.0]]),
        np.array([[1.0]]),
        np.zeros((1, 1)), 0.0
    )
    sys_i = ctrl.c2d(sys_i_c, Ts_i, ctrl.C2dMethod.ZOH)

    # Inner PID
    pp_i = ctrl.PIDParams()
    pp_i.Kp = 5.0; pp_i.Ki = 20.0; pp_i.Kd = 0.05
    pp_i.uMin = -20.0; pp_i.uMax = 20.0
    pid_inner = ctrl.DiscretePID(pp_i, Ts_i)

    # Outer slow plant G_o(s) = 1/(s+0.5)
    sys_o_c = ctrl.StateSpace(
        np.array([[-0.5]]),
        np.array([[0.5]]),
        np.array([[1.0]]),
        np.zeros((1, 1)), 0.0
    )
    sys_o = ctrl.c2d(sys_o_c, Ts_o, ctrl.C2dMethod.ZOH)

    # Outer MPC
    mp = ctrl.MPCParams()
    mp.Np = 15; mp.Nc = 4; mp.rho_y = 1.0; mp.rho_u = 0.5
    mpc_outer = ctrl.DiscreteMPC(sys_o, mp)

    x_i = np.zeros(sys_i.state_size())
    x_o = np.zeros(sys_o.state_size())
    y_i, y_o = 0.0, 0.0
    ref = 1.0
    N_outer = 400

    for ko in range(N_outer):
        inner_sp = mpc_outer.compute(ref - y_o)

        for ki in range(10):
            e_i = inner_sp - y_i
            u_i = pid_inner.compute(e_i)
            y_i_vec, x_i = ctrl.ss_step_copy(sys_i, x_i, np.array([u_i]))
            y_i = float(np.squeeze(y_i_vec))

        y_o_vec, x_o = ctrl.ss_step_copy(sys_o, x_o, np.array([y_i]))
        y_o = float(np.squeeze(y_o_vec))

    print(f"CASCADE PID(inner)+MPC(outer): final y_outer={y_o:.4f}  (ref={ref})")
    assert abs(y_o - ref) < 0.05, f"Did not converge: y_o={y_o:.4f}"
    print("PASS")


if __name__ == "__main__":
    main()
