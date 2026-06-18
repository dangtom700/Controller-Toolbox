"""
ex69_fuzzy_chattering_smc.py
ADDITIVE: FuzzyPD correction to smooth SMC chattering.

SMC provides robust control; FuzzyPD adds a smooth continuous correction
near the sliding surface, reducing chattering without losing robustness.
"""

import _setup_bindings  # noqa: F401
import ctrl_toolbox as ctrl
import numpy as np


def main():
    Ts = 0.01

    # SMC (primary)
    smc_p = ctrl.SMCParams()
    smc_p.c_e  = 1.0
    smc_p.c_de = 5.0 * Ts   # lambda=5 [1/s] -> c_de=lambda*Ts
    smc_p.K    = 5.0         # switching gain
    smc_p.phi  = 0.1
    smc_p.uMin = -10.0
    smc_p.uMax =  10.0
    smc = ctrl.DiscreteSMC(smc_p, Ts)

    # FuzzyPD (additive chattering reduction)
    fpdp = ctrl.FuzzyPDParams()
    fpdp.e_scale  = 0.5
    fpdp.de_scale = 0.05
    fpdp.u_scale  = 0.8
    fuzzy = ctrl.FuzzyPD(fpdp, Ts)

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
    energy = 0.0

    for k in range(N):
        e = ref - y
        u_smc   = smc.compute(y - ref)   # SMC: compute(y-ref)
        u_fuzzy = fuzzy.compute(e)
        u_total = u_smc + 0.3 * u_fuzzy

        y_vec, x = ctrl.ss_step_copy(sys, x, np.array([u_total]))
        y = float(np.squeeze(y_vec))
        energy += u_total**2 * Ts

    print(f"ADDITIVE FuzzyPD+SMC: final y={y:.4f}  energy={energy:.4f}")
    assert abs(y - ref) < 0.05 and np.isfinite(energy), \
        f"Did not converge: y={y:.4f}"
    print("PASS")


if __name__ == "__main__":
    main()
