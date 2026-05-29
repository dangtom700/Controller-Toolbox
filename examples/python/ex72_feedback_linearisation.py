"""
ex72_feedback_linearisation.py
Demonstrates FeedbackLinearisationController via the pybind11 binding.

System 1 - Cubic drift:  xdot = -x^3 + u
  f(x, u_prev) = -x[0]^3,  g(x, u_prev) = 1.0
  Inner PID: Kp=5, Ki=2
  Target: x(0) -> 1.0 from x=0 in 500 steps (Ts=0.01).

System 2 - Nonlinear pendulum:  xdot1=x2, xdot2=-(g/L)sin(x1)-b.x2+u
  f = -(g/L)sin(x[0]) - b.x[1],  g = 1.0
  Inner PID: Kp=20, Ki=5, Kd=3
  Target: x1 -> pi/6 from x=(0,0) in 1000 steps.
"""

import math
import _setup_bindings  # noqa: F401
import ctrl_toolbox as ctrl
import numpy as np


def main():
    Ts = 0.01

    # =========================================================================
    # System 1: xdot = -x^3 + u
    # =========================================================================
    def f1(x, u_prev):
        return -float(x[0])**3

    def g1(x, u_prev):
        return 1.0

    pp1 = ctrl.PIDParams()
    pp1.Kp = 5.0; pp1.Ki = 2.0; pp1.Kd = 0.0; pp1.N = 100.0
    pp1.uMin = -100.0; pp1.uMax = 100.0
    inner1 = ctrl.DiscretePID(pp1, Ts)

    flp1 = ctrl.FLParams()
    flp1.uMin = -50.0; flp1.uMax = 50.0; flp1.regularisation_eps = 1e-6

    fl1 = ctrl.FeedbackLinearisationController(f1, g1, inner1, flp1, Ts)

    ref1 = 1.0
    x1 = np.array([0.0])
    for _ in range(500):
        fl1.set_state(x1)
        u = fl1.compute(ref1 - float(x1[0]))
        x1[0] += Ts * (-x1[0]**3 + u)

    err1 = abs(float(x1[0]) - ref1)
    print(f"System 1 (xdot=-x^3+u): y={float(x1[0]):.4f}  |err|={err1:.4f}  (need < 0.05)")
    assert err1 < 0.05 and math.isfinite(float(x1[0])), \
        f"FAIL: cubic drift did not converge: y={float(x1[0]):.4f}"

    # =========================================================================
    # System 2: Nonlinear pendulum
    # =========================================================================
    grav, L, b_damp = 9.81, 1.0, 0.5

    def f2(x, u_prev):
        return -(grav / L) * math.sin(float(x[0])) - b_damp * float(x[1])

    def g2(x, u_prev):
        return 1.0

    # Gains for double-integrator inner plant (relative degree 2):
    # poles at {-1, -2+j, -2-j} -> Kd=5, Kp=9, Ki=5
    pp2 = ctrl.PIDParams()
    pp2.Kp = 9.0; pp2.Ki = 5.0; pp2.Kd = 5.0; pp2.N = 100.0
    pp2.uMin = -50.0; pp2.uMax = 50.0
    inner2 = ctrl.DiscretePID(pp2, Ts)

    flp2 = ctrl.FLParams()
    flp2.uMin = -30.0; flp2.uMax = 30.0; flp2.regularisation_eps = 1e-6

    fl2 = ctrl.FeedbackLinearisationController(f2, g2, inner2, flp2, Ts)

    ref2 = math.pi / 6.0   # 30 degrees
    x2 = np.array([0.0, 0.0])
    for _ in range(2000):   # 20 s - 20 time constants for dominant pole tau=1s
        fl2.set_state(x2)
        u = fl2.compute(ref2 - float(x2[0]))
        xdot0 = float(x2[1])
        xdot1 = -(grav / L) * math.sin(float(x2[0])) - b_damp * float(x2[1]) + u
        x2[0] += Ts * xdot0
        x2[1] += Ts * xdot1

    err2 = abs(float(x2[0]) - ref2)
    print(f"System 2 (pendulum): theta={math.degrees(float(x2[0])):.2f} deg  "
          f"ref={math.degrees(ref2):.1f} deg  |err|={err2:.4f} rad  (need < 0.05)")
    assert err2 < 0.05 and math.isfinite(float(x2[0])), \
        f"FAIL: pendulum did not converge: theta={math.degrees(float(x2[0])):.2f} deg"

    print("PASS")


if __name__ == "__main__":
    main()
