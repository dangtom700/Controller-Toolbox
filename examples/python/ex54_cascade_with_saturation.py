"""
ex54_cascade_with_saturation.py
Cascade PID with anti-windup on the inner loop using ctrl_toolbox binding.

Inner loop: fast temperature control (tau=0.5 s, actuator limit +/-5)
Outer loop: slow setpoint from an outer PID
"""

import _setup_bindings  # noqa: F401
import ctrl_toolbox as ctrl
import numpy as np


def main():
    Ts = 0.05

    # Inner PID with actuator limits (anti-windup active)
    pp_inner = ctrl.PIDParams()
    pp_inner.Kp    = 3.0
    pp_inner.Ki    = 2.0
    pp_inner.Kd    = 0.05
    pp_inner.uMin = -5.0
    pp_inner.uMax =  5.0
    pid_inner = ctrl.DiscretePID(pp_inner, Ts)

    # Outer PID (no hard limits needed for this example)
    pp_outer = ctrl.PIDParams()
    pp_outer.Kp = 0.8
    pp_outer.Ki = 0.2
    pp_outer.Kd = 0.0
    pid_outer = ctrl.DiscretePID(pp_outer, Ts)

    tau_inner = 0.5
    tau_outer = 2.0
    K_plant   = 1.0

    a_i = np.exp(-Ts / tau_inner)
    a_o = np.exp(-Ts / tau_outer)

    y_inner, y_outer = 0.0, 0.0
    outer_ref = 1.0
    N = 800

    for k in range(N):
        e_outer  = outer_ref - y_outer
        inner_sp = pid_outer.compute(e_outer)

        e_inner = inner_sp - y_inner
        u       = pid_inner.compute(e_inner)  # saturated by pp_inner.u_max

        y_inner = a_i * y_inner + (1.0 - a_i) * K_plant * u
        y_outer = a_o * y_outer + (1.0 - a_o) * K_plant * y_inner

    print(f"Cascade with saturation: final y_outer = {y_outer:.4f} (ref={outer_ref})")
    assert abs(y_outer - outer_ref) < 0.05, \
        f"Cascade did not converge: y_outer={y_outer:.4f}"
    print("PASS")


if __name__ == "__main__":
    main()
