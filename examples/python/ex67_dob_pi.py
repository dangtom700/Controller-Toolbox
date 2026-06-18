"""
ex67_dob_pi.py
OBSERVER + STATE FEEDBACK: Disturbance Observer (DOB) + PI.

DOB estimates and cancels matched disturbance on the plant.
PI provides setpoint tracking.
"""

import _setup_bindings  # noqa: F401
import ctrl_toolbox as ctrl
import numpy as np


def main():
    Ts = 0.05

    # PI controller
    pp = ctrl.PIDParams()
    pp.Kp = 1.5; pp.Ki = 0.8; pp.Kd = 0.0
    pp.uMin = -10.0; pp.uMax = 10.0
    pi = ctrl.DiscretePID(pp, Ts)

    # Nominal plant: G_nom(s)=1/(s+1), ZOH
    sys_c = ctrl.StateSpace(
        np.array([[-1.0]]),
        np.array([[1.0]]),
        np.array([[1.0]]),
        np.zeros((1, 1)), 0.0)
    sys_nom = ctrl.c2d(sys_c, Ts, ctrl.C2dMethod.ZOH)
    x_nom = np.zeros(sys_nom.state_size())

    # True plant: G_true = 1.5/(s+0.8)
    K_true, tau_true = 1.5, 1.0/0.8
    a_true = np.exp(-Ts/tau_true)

    # Q-filter: 1st-order LP at omega_q=5 rad/s
    omega_q = 5.0
    q_a = np.exp(-omega_q * Ts)
    G_nom_DC = 1.0  # DC gain of nominal plant

    y_true, x_q = 0.0, 0.0
    ref = 1.0
    N = 600

    for k in range(N):
        u_pi = pi.compute(ref - y_true)

        y_nom_vec, x_nom = ctrl.ss_step_copy(sys_nom, x_nom, np.array([u_pi]))
        y_nom = float(np.squeeze(y_nom_vec))

        x_q = q_a * x_q + (1 - q_a) * (y_true - y_nom)
        d_hat = x_q / G_nom_DC

        u_total = u_pi - d_hat
        d = 0.5 if k >= 100 else 0.0
        y_true = a_true * y_true + (1-a_true)*K_true*u_total + d

    print(f"DOB+PI: final y={y_true:.4f}  (ref={ref})")
    assert abs(y_true - ref) < 0.1 and np.isfinite(y_true), \
        f"Did not converge: y={y_true:.4f}"
    print("PASS")


if __name__ == "__main__":
    main()
