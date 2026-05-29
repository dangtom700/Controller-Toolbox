"""
ex70_leadlag_integrator.py
ADDITIVE: LeadLag (phase advance) + pure integrator (I-only controller).

Pure integral action gives zero steady-state error but poor phase margin.
Lead-Lag additive correction restores phase margin at crossover.
"""

import _setup_bindings  # noqa: F401
import ctrl_toolbox as ctrl
import numpy as np


def main():
    Ts = 0.02

    # Lead-Lag compensator
    ll_p = ctrl.LeadLagParams()
    ll_p.continuous_zero = 0.7   # zero frequency [rad/s]
    ll_p.continuous_pole = 3.5   # pole frequency [rad/s] (lead ratio ~5)
    ll_p.gain            = 1.2
    lead_lag = ctrl.DiscreteLeadLag(ll_p, Ts)

    # Plant: G(s) = 1/(s+0.2)
    sys_c = ctrl.StateSpace(
        np.array([[-0.2]]),
        np.array([[0.2]]),
        np.array([[1.0]]),
        np.zeros((1, 1)), 0.0)
    sys = ctrl.c2d(sys_c, Ts, ctrl.C2dMethod.ZOH)
    x = np.zeros(sys.state_size())
    y, ref = 0.0, 1.0
    Ki = 0.8
    integral = 0.0
    N = 800

    for k in range(N):
        e = ref - y
        integral += e * Ts
        u_I  = Ki * integral
        u_ll = lead_lag.compute(e)
        u = u_I + 0.5 * u_ll

        y_vec, x = ctrl.ss_step_copy(sys, x, np.array([u]))
        y = float(np.squeeze(y_vec))

    print(f"ADDITIVE LeadLag+Integrator: final y={y:.4f}  (ref={ref})")
    assert abs(y - ref) < 0.25, f"Did not converge: y={y:.4f}"
    print("PASS")


if __name__ == "__main__":
    main()
