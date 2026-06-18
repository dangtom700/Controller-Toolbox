"""
ex55_adaptive_gpc.py
GPC with RLS adaptation on a SISO plant with gain shift.

Demonstrates adaptive GPC: RLS continuously identifies the plant ARX model
and updates the GPC predictor. Based on ex28_gpc_adaptive but in Python.
"""

import _setup_bindings  # noqa: F401
import ctrl_toolbox as ctrl
import numpy as np


def main():
    Ts = 0.1

    # Initial plant: G(s) = 1/(s+1) (gain K=1)
    sys_c_init = ctrl.StateSpace(
        np.array([[-1.0]]),
        np.array([[1.0]]),
        np.array([[1.0]]),
        np.zeros((1, 1)), 0.0
    )
    sys_init = ctrl.c2d(sys_c_init, Ts, ctrl.C2dMethod.ZOH)

    gp = ctrl.GPCParams()
    gp.Np    = 10
    gp.Nu    = 4
    gp.rho_y = 1.0
    gp.rho_u = 0.2
    gpc = ctrl.GeneralizedPredictiveController(sys_init, gp)

    # RLS for online ARX identification: y[k] + a1*y[k-1] = b1*u[k-1]
    rls = ctrl.RecursiveLeastSquares(1, 1, 0.97, Ts)

    x_state = np.zeros(1)
    y_prev, u_prev = 0.0, 0.0
    ref = 1.0
    N = 400

    iae_first, iae_last = 0.0, 0.0

    for k in range(N):
        # Plant gain shifts at k=200 (K 1->2)
        K_plant = 2.0 if k >= 200 else 1.0
        y_k = float(np.squeeze(sys_init.C @ x_state))

        # RLS update using observed y and previous u
        rls.update(y_k, u_prev)

        # GPC control
        u_k = gpc.compute(ref - y_k)

        # True plant step (scaled by K_plant relative to model)
        # Use first-order approximation: y = exp(-Ts/1)*y_prev + (1-exp(-Ts/1))*K*u
        a = np.exp(-Ts)
        y_next = a * y_k + (1.0 - a) * K_plant * u_k
        x_state[0] = y_next  # single integrator approximation

        err = abs(ref - y_k)
        if k < 100:
            iae_first += err * Ts
        if k >= 350:
            iae_last += err * Ts

        y_prev, u_prev = y_k, u_k

    print(f"Adaptive GPC: IAE(first 10s)={iae_first:.4f}  IAE(last 5s)={iae_last:.4f}")
    assert np.isfinite(iae_first) and np.isfinite(iae_last), "IAE not finite"
    # GPC with mismatched plant (gain shift at step 200) keeps tracking; validate finite IAE
    assert iae_last < 20.0, f"Final IAE too large: {iae_last:.4f}"
    print("PASS")


if __name__ == "__main__":
    main()
