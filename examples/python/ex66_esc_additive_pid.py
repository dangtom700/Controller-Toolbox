"""
ex66_esc_additive_pid.py
ADDITIVE: ExtremumSeeker + PID.

ExtremumSeeker slowly optimises the operating setpoint.
PID provides fast setpoint tracking.
"""

import _setup_bindings  # noqa: F401
import ctrl_toolbox as ctrl
import numpy as np


def main():
    Ts = 0.01

    sys_c = ctrl.StateSpace(
        np.array([[-1.0]]),
        np.array([[1.0]]),
        np.array([[1.0]]),
        np.zeros((1, 1)), 0.0)
    sys = ctrl.c2d(sys_c, Ts, ctrl.C2dMethod.ZOH)

    pp = ctrl.PIDParams()
    pp.Kp = 3.0; pp.Ki = 2.0; pp.Kd = 0.0
    pp.uMin = -10.0; pp.uMax = 10.0
    pid = ctrl.DiscretePID(pp, Ts)

    esc_p = ctrl.ExtremumSeekerParams()
    esc_p.perturb_freq  = 0.5
    esc_p.perturb_amp   = 0.05
    esc_p.integ_gain    = 0.3
    esc_p.seek_minimum  = True
    esc = ctrl.ExtremumSeeker(esc_p, Ts)

    x = np.zeros(sys.state_size())
    y, r_esc = 0.0, 0.5
    N = 2000

    for k in range(N):
        J = (y - 0.8)**2
        dr_esc = esc.compute(J)
        r_esc += Ts * dr_esc * 0.1

        u = pid.compute(r_esc - y)
        y_vec, x = ctrl.ss_step_copy(sys, x, np.array([u]))
        y = float(np.squeeze(y_vec))

    print(f"ADDITIVE ESC+PID: final y={y:.4f}  setpoint={r_esc:.4f}")
    assert np.isfinite(y) and abs(y) < 5.0, f"System diverged: y={y:.4f}"
    print("PASS")


if __name__ == "__main__":
    main()
