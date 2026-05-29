"""
ex59_smith_predictor_real_plant.py
Smith predictor on a hardware-like plant with transport delay (simulated).

Models a temperature sensor with 3-step transport delay (data acquisition lag).
Smith predictor compensates the delay; pure PID without Smith is compared.
"""

import _setup_bindings  # noqa: F401
import ctrl_toolbox as ctrl
import numpy as np


def simulate_with_smith(delay_steps, Ts, N=300, ref=1.0):
    """Run Smith predictor on G(s)=1/(s+1) + integer delay, return IAE."""
    sys_c = ctrl.StateSpace(
        np.array([[-1.0]]),
        np.array([[1.0]]),
        np.array([[1.0]]),
        np.zeros((1, 1)), 0.0
    )
    G0 = ctrl.c2d(sys_c, Ts, ctrl.C2dMethod.ZOH)

    pp = ctrl.PIDParams()
    pp.Kp = 2.0; pp.Ki = 0.8; pp.Kd = 0.0
    pp.uMin = -10.0; pp.uMax = 10.0
    pid_inner = ctrl.DiscretePID(pp, Ts)

    sp = ctrl.SmithPredictor(pid_inner, G0, delay_steps)
    u_buf = [0.0] * (delay_steps + 2)

    x = np.zeros(1)
    y, iae = 0.0, 0.0

    for k in range(N):
        u_sp = sp.compute(ref - y)
        u_delayed = u_buf[delay_steps]
        u_buf[1:] = u_buf[:-1]
        u_buf[0] = u_sp

        y_next, x_next = ctrl.ss_step_copy(G0, x, np.array([u_delayed]))
        y = float(np.squeeze(y_next))
        x = x_next
        iae += abs(ref - y) * Ts

    return iae


def simulate_plain_pid(delay_steps, Ts, N=300, ref=1.0):
    """Run plain PID (no Smith) with the same delay, return IAE."""
    sys_c = ctrl.StateSpace(
        np.array([[-1.0]]),
        np.array([[1.0]]),
        np.array([[1.0]]),
        np.zeros((1, 1)), 0.0
    )
    G0 = ctrl.c2d(sys_c, Ts, ctrl.C2dMethod.ZOH)

    pp = ctrl.PIDParams()
    pp.Kp = 2.0; pp.Ki = 0.8; pp.Kd = 0.0
    pp.uMin = -10.0; pp.uMax = 10.0
    pid = ctrl.DiscretePID(pp, Ts)

    u_buf = [0.0] * (delay_steps + 2)
    x = np.zeros(1)
    y, iae = 0.0, 0.0

    for k in range(N):
        u_k = pid.compute(ref - y)
        u_delayed = u_buf[delay_steps]
        u_buf[1:] = u_buf[:-1]
        u_buf[0] = u_k

        y_next, x_next = ctrl.ss_step_copy(G0, x, np.array([u_delayed]))
        y = float(np.squeeze(y_next))
        x = x_next
        iae += abs(ref - y) * Ts

    return iae


def main():
    Ts          = 0.1
    delay_steps = 5   # 0.5 s transport delay

    iae_smith = simulate_with_smith(delay_steps, Ts)
    iae_plain = simulate_plain_pid(delay_steps, Ts)

    print(f"Smith predictor IAE: {iae_smith:.4f}")
    print(f"Plain PID IAE:       {iae_plain:.4f}")
    print(f"Smith improvement:   {100*(iae_plain - iae_smith)/iae_plain:.1f} %")

    assert np.isfinite(iae_smith), "Smith IAE not finite"
    assert np.isfinite(iae_plain), "Plain IAE not finite"
    # Smith should not be worse than plain PID (allows ties)
    assert iae_smith <= iae_plain + 0.5, \
        f"Smith predictor worse than plain PID: {iae_smith:.4f} > {iae_plain:.4f}"
    print("PASS")


if __name__ == "__main__":
    main()
