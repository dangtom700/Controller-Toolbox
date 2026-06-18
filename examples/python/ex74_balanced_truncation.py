"""
ex74_balanced_truncation.py
Balanced truncation model order reduction via pybind11 binding.

4th-order continuous plant (poles at -1, -5, -20, -100 rad/s) ZOH-discretised
at Ts=0.01. Truncate to order 2. Verify error bound and stability of reduced model.
"""

import _setup_bindings  # noqa: F401
import ctrl_toolbox as ctrl
import numpy as np


def main():
    Ts = 0.01

    # Build 4th-order plant in controllable canonical form:
    # G(s) = 1 / ((s+1)(s+5)(s+20)(s+100))
    # Denominator: s^4 + 126s^3 + 2625s^2 + 12600s + 10000
    Ac = np.array([[ 0,      1,       0,     0],
                   [ 0,      0,       1,     0],
                   [ 0,      0,       0,     1],
                   [-10000, -12600, -2625, -126]], dtype=float)
    Bc = np.array([[0], [0], [0], [1]], dtype=float)
    Cc = np.array([[10000, 0, 0, 0]], dtype=float)
    Dc = np.zeros((1, 1))

    sys_c = ctrl.StateSpace(Ac, Bc, Cc, Dc, 0.0)
    sys_d = ctrl.c2d(sys_c, Ts, ctrl.C2dMethod.ZOH)

    # Balanced truncation to r=2
    res2 = ctrl.balanced_truncate(sys_d, 2)

    print(f"Hankel singular values: {res2.hankel_singular_values}")
    print(f"Error bound (r=2): {res2.error_bound:.6e}")
    print(f"Reduced model stable: {res2.is_stable}")

    assert res2.is_stable, "Reduced model is not stable"
    assert res2.error_bound < res2.hankel_singular_values[0], \
        "Error bound >= sigma1 (truncation ineffective)"

    # Check that HSVs are descending
    hsv = res2.hankel_singular_values
    for i in range(len(hsv) - 1):
        assert hsv[i] >= hsv[i+1], f"HSVs not descending at i={i}"

    # suggestOrder
    r_best = ctrl.suggest_order(res2, 0.01)
    print(f"Suggested order for 1% tol: {r_best}")
    assert 1 <= r_best <= 3, f"Unexpected suggested order: {r_best}"

    # DC gain of reduced model should be close to full model
    n_full = sys_d.state_size()
    n_red  = res2.reduced.state_size()
    A_r, B_r, C_r = res2.reduced.A, res2.reduced.B, res2.reduced.C

    dc_full    = float(np.squeeze(Cc @ np.linalg.solve(np.eye(n_full) - sys_d.A, sys_d.B)))
    dc_reduced = float(np.squeeze(C_r @ np.linalg.solve(np.eye(n_red) - A_r, B_r)))
    print(f"DC gain - full: {dc_full:.4f}  reduced: {dc_reduced:.4f}")
    assert abs(dc_full - dc_reduced) < res2.error_bound + 0.01, \
        f"DC gain deviation exceeds error bound"

    print("PASS")


if __name__ == "__main__":
    main()
