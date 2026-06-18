"""
ex68_bumpless_transfer.py
SUPERVISORY: Bumpless transfer between coarse and fine PID controllers.

Demonstrates ControllerStack::Supervisory switching without actuator bump.
Coarse PID handles large errors; fine PID takes over near the setpoint.
"""

import _setup_bindings  # noqa: F401
import ctrl_toolbox as ctrl
import numpy as np


def main():
    Ts = 0.05

    # Coarse PID: aggressive
    pp_c = ctrl.PIDParams()
    pp_c.Kp = 4.0; pp_c.Ki = 1.5; pp_c.Kd = 0.1
    pp_c.uMin = -10.0; pp_c.uMax = 10.0

    # Fine PID: conservative
    pp_f = ctrl.PIDParams()
    pp_f.Kp = 1.2; pp_f.Ki = 0.5; pp_f.Kd = 0.02
    pp_f.uMin = -10.0; pp_f.uMax = 10.0

    pid_c = ctrl.DiscretePID(pp_c, Ts)
    pid_f = ctrl.DiscretePID(pp_f, Ts)

    # ControllerStack: Supervisory mode
    stack = ctrl.ControllerStack(ctrl.StackMode.Supervisory, Ts)
    stack.add_controller(pid_c, "coarse", weight=1.0, condition=lambda e, u: abs(e) >= 0.15)
    stack.add_controller(pid_f, "fine",   weight=1.0, condition=lambda e, u: abs(e) < 0.15)

    # Plant
    sys_c = ctrl.StateSpace(
        np.array([[-1.0]]),
        np.array([[1.0]]),
        np.array([[1.0]]),
        np.zeros((1, 1)), 0.0)
    sys = ctrl.c2d(sys_c, Ts, ctrl.C2dMethod.ZOH)
    x = np.zeros(sys.state_size())
    y, ref = 0.0, 1.0
    N = 600
    u_prev, last_ctrl = 0.0, ""
    max_bump = 0.0

    for k in range(N):
        e = ref - y
        u = stack.compute(e)
        active = stack.active_controller_name()
        if active != last_ctrl and k > 0:
            jump = abs(u - u_prev)
            max_bump = max(max_bump, jump)
            print(f"  Switch at k={k}: {last_ctrl}->{active}  |u_jump|={jump:.4f}  e={e:.4f}")
        last_ctrl = active

        y_vec, x = ctrl.ss_step_copy(sys, x, np.array([u]))
        y = float(np.squeeze(y_vec))
        u_prev = u

    print(f"Bumpless supervisory: final y={y:.4f}  (ref={ref})")
    print(f"Max |u| jump on switch = {max_bump:.4f}")
    assert abs(y - ref) < 0.05, f"Did not converge: y={y:.4f}"
    print("PASS")


if __name__ == "__main__":
    main()
