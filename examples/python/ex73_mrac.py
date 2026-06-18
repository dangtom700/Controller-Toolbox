"""
ex73_mrac.py
Model Reference Adaptive Control (MRAC) via pybind11 binding.

Plant: y[k+1] = 0.7*y[k] + b_p*u[k]
  Phase 1 (k=0..499): b_p = 0.5  (nominal)
  Phase 2 (k=500..999): b_p = 0.9 (gain jump +80%)

Reference model: a_m=0.5, b_m=0.5 (faster closed-loop response)
Reference: r = 1.0 (step)

PASS when:
  Phase 1 |e_m| < 0.05 at k=499
  Phase 2 |e_m| < 0.10 at k=999
  Parameters stay bounded at all times
"""

import _setup_bindings  # noqa: F401
import ctrl_toolbox as ctrl
import numpy as np
import math


def run_phase(b_p_values, n_steps):
    """Run MRAC for n_steps with b_p changing at step 500."""
    mp = ctrl.MRACParams()
    mp.a_m = 0.5
    mp.b_m = 0.5
    mp.gamma_r = 3.0
    mp.gamma_y = 1.5
    mp.sigma = 0.02
    mp.theta_max = 20.0

    mrac = ctrl.MRACController(mp, 0.1)
    y = 0.0
    em_history = []
    b_p = b_p_values[0]

    for k in range(n_steps):
        if k < len(b_p_values):
            b_p = b_p_values[k]
        mrac.set_reference(1.0)
        u = mrac.compute(y)
        y = 0.7 * y + b_p * u
        em_history.append(mrac.model_error())

        # Check parameter bound
        norm = math.sqrt(mrac.theta_r()**2 + mrac.theta_y()**2)
        assert norm <= mp.theta_max + 1e-6, \
            f"||theta|| = {norm:.4f} > theta_max at k={k}"

    return em_history, mrac


def main():
    # Phase 1: nominal gain b_p = 0.5
    b_p_seq = [0.5] * 500 + [0.9] * 500

    em_hist, mrac_final = run_phase(b_p_seq, 1000)

    em_phase1 = abs(em_hist[499])
    em_phase2 = abs(em_hist[999])

    print(f"Phase 1 |e_m| at k=499: {em_phase1:.4f}  (need < 0.05)")
    print(f"Phase 2 |e_m| at k=999: {em_phase2:.4f}  (need < 0.10)")
    print(f"Final: theta_r={mrac_final.theta_r():.4f}  theta_y={mrac_final.theta_y():.4f}")

    assert em_phase1 < 0.05, f"Phase 1 did not adapt: |e_m|={em_phase1:.4f}"
    assert em_phase2 < 0.10, f"Phase 2 did not re-adapt: |e_m|={em_phase2:.4f}"
    print("PASS")


if __name__ == "__main__":
    main()
