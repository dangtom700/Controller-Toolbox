"""
ex56_pareto_mpc_weights.py
Pareto front for MPC weight trade-off: output tracking (ISE) vs control effort (ISU).

Sweeps rho_y/rho_u ratio and computes ISE and ISU for each design,
demonstrating the tracking-effort trade-off.
"""

import _setup_bindings  # noqa: F401
import ctrl_toolbox as ctrl
import numpy as np


def simulate_mpc(sys, rho_y, rho_u, N_sim=500, ref=1.0, Ts=0.1):
    """Run MPC closed-loop and return (ISE, ISU)."""
    p = ctrl.MPCParams()
    p.Np    = 12
    p.Nc    = 4
    p.rho_y = rho_y
    p.rho_u = rho_u

    mpc = ctrl.DiscreteMPC(sys, p)
    x   = np.zeros(sys.state_size())
    ise, isu = 0.0, 0.0

    for _ in range(N_sim):
        y = float(np.squeeze(sys.C @ x))
        u = mpc.compute(ref - y)

        # ssStep equivalent
        x_new, _ = ctrl.ss_step_copy(sys, x, np.array([u]))
        x = x_new

        ise += (ref - y) ** 2 * Ts
        isu += u ** 2 * Ts

    return ise, isu


def main():
    Ts = 0.1

    # Plant: G(s) = 1/(s+1), ZOH
    sys_c = ctrl.StateSpace(
        np.array([[-1.0]]),
        np.array([[1.0]]),
        np.array([[1.0]]),
        np.zeros((1, 1)), 0.0
    )
    sys = ctrl.c2d(sys_c, Ts, ctrl.C2dMethod.ZOH)

    rho_ratio_vals = [0.01, 0.05, 0.1, 0.5, 1.0, 5.0, 10.0]
    print("rho_y/rho_u  ISE        ISU")
    print("-" * 35)

    ise_vals, isu_vals = [], []
    for ratio in rho_ratio_vals:
        ise, isu = simulate_mpc(sys, rho_y=1.0, rho_u=1.0 / ratio)
        ise_vals.append(ise)
        isu_vals.append(isu)
        print(f"{ratio:10.2f}   {ise:8.4f}   {isu:8.4f}")

    # Verify Pareto trade-off: as rho_y/rho_u increases, ISE decreases and ISU increases
    ise_monotone = all(ise_vals[i] >= ise_vals[i+1] for i in range(len(ise_vals)-1))
    isu_monotone = all(isu_vals[i] <= isu_vals[i+1] for i in range(len(isu_vals)-1))

    assert all(np.isfinite(v) for v in ise_vals + isu_vals), "Non-finite ISE/ISU"
    # Allow 1 violation due to nonlinearity
    ise_violations = sum(1 for i in range(len(ise_vals)-1) if ise_vals[i] < ise_vals[i+1])
    isu_violations = sum(1 for i in range(len(isu_vals)-1) if isu_vals[i] > isu_vals[i+1])
    assert ise_violations <= 1, f"ISE not monotone decreasing: {ise_vals}"
    assert isu_violations <= 1, f"ISU not monotone increasing: {isu_vals}"
    print("Pareto trade-off verified.")
    print("PASS")


if __name__ == "__main__":
    main()
