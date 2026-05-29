"""
ex63_adrc_inner_pid_outer.py
CASCADE: ADRC (inner, ESO corrector) + PID (outer).

ADRC's Extended State Observer cancels disturbances on the inner plant,
making it behave like a clean integrator for the outer PID.
"""

import _setup_bindings  # noqa: F401
import ctrl_toolbox as ctrl
import numpy as np


def main():
    Ts = 0.01

    # ADRC inner controller
    adrc_p = ctrl.ADRCParams()
    adrc_p.b0      = 1.0
    adrc_p.omega_o = 10.0   # ESO bandwidth (was omega_n)
    adrc_p.omega_c = 4.0
    adrc_p.uMin    = -15.0
    adrc_p.uMax    =  15.0
    adrc = ctrl.DiscreteADRC(adrc_p, Ts)

    # Outer PID
    pp_outer = ctrl.PIDParams()
    pp_outer.Kp = 1.2; pp_outer.Ki = 0.4; pp_outer.Kd = 0.02
    pp_outer.uMin = -8.0; pp_outer.uMax = 8.0
    pid_outer = ctrl.DiscretePID(pp_outer, Ts)

    # 2nd-order inner plant with disturbance
    wn, zeta = 3.0, 0.6
    a1 = 2.0 * zeta * wn
    a2 = wn * wn
    y_i, dy_i = 0.0, 0.0
    y_o = 0.0
    ref_outer = 1.0
    N = 1200

    for k in range(N):
        inner_sp = pid_outer.compute(ref_outer - y_o)
        u_adrc = adrc.compute(inner_sp - y_i)

        d = 0.4 if k > 400 else 0.0
        ddy_i = -a1*dy_i - a2*y_i + a2*u_adrc + a2*d
        dy_i += Ts * ddy_i
        y_i += Ts * dy_i

        if k % 10 == 0:
            y_o = 0.95 * y_o + 0.05 * y_i

    print(f"CASCADE ADRC(inner)+PID(outer): final y_outer={y_o:.4f}  (ref={ref_outer})")
    assert abs(y_o - ref_outer) < 0.15, f"Did not converge: y_o={y_o:.4f}"
    print("PASS")


if __name__ == "__main__":
    main()
