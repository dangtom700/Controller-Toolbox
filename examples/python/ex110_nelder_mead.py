"""
ex110_nelder_mead.py

Phase 3 (MO2): NelderMead finds PID gains minimising IAE, no bounds needed.

Mirrors ex93_nelder_mead.cpp - a quick 3-parameter PID retune where NelderMead needs
only an initial point, compared against AutoTuner's (CMA-ES) evaluation budget on the
same problem.
"""

import sys
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'NelderMead'):
        raise AttributeError("NelderMead not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

Ts = 0.05
N_sim = 100
ref_val = 1.0

A_c = np.array([[0.0, 1.0], [-1.0, -2.0]])
B_c = np.array([[0.0], [1.0]])
C_c = np.array([[1.0, 0.0]])
D_c = np.array([[0.0]])
sys_c = ctrl.StateSpace(A_c, B_c, C_c, D_c, 0.0)
plant = ctrl.c2d(sys_c, Ts, ctrl.C2dMethod.ZOH)


def simulate_iae(params):
    pp = ctrl.PIDParams()
    pp.Kp, pp.Ki, pp.Kd = float(params[0]), float(params[1]), float(params[2])
    pp.N = 10.0
    pp.uMin, pp.uMax = -10.0, 10.0
    pid = ctrl.DiscretePID(pp, Ts)

    x = np.zeros(plant.A.shape[0])
    iae = 0.0
    for _ in range(N_sim):
        y = float((plant.C @ x)[0])
        e = ref_val - y
        u = pid.compute(e)
        x = plant.A @ x + plant.B[:, 0] * u
        iae += abs(e) * Ts
    return iae


x_default = np.array([1.0, 0.1, 0.0])
iae_default = simulate_iae(x_default)
print(f"Default gains  Kp={x_default[0]:.2f} Ki={x_default[1]:.2f} Kd={x_default[2]:.2f}  "
      f"IAE={iae_default:.4f}")

nmp = ctrl.NelderMeadParams()
nmp.n_dim = 3
nm = ctrl.NelderMead(nmp)
result = nm.optimize(simulate_iae, x_default)

iae_tuned = result.cost
print(f"Tuned  gains   Kp={result.params[0]:.3f} Ki={result.params[1]:.3f} "
      f"Kd={result.params[2]:.3f}  IAE={iae_tuned:.4f}")
print(f"Evaluations={result.n_evals}  Iterations={result.n_gens}  Converged={result.converged}")
print(f"IAE improvement: {(iae_default - iae_tuned) / iae_default * 100.0:.1f}%")

atp = ctrl.AutoTunerParams()
atp.n = 3
atp.lower = np.array([0.01, 0.0, 0.0])
atp.upper = np.array([5.0, 2.0, 1.0])
tuner = ctrl.AutoTuner(atp)
at_result = tuner.tune(simulate_iae, x_default)
print(f"AutoTuner (CMA-ES) for comparison: Evaluations={at_result.n_evals}  IAE={at_result.cost:.4f}")

improved = iae_tuned < iae_default * 0.9
finite = np.isfinite(iae_tuned) and np.all(np.isfinite(result.params))
fewer_evals = result.n_evals < at_result.n_evals

ok = improved and finite and fewer_evals
if not improved:
    print(f"[FAIL] tuned IAE {iae_tuned:.4f} not < 0.9 * default {iae_default * 0.9:.4f}")
if not fewer_evals:
    print(f"[FAIL] NelderMead used {result.n_evals} evals, not fewer than AutoTuner's {at_result.n_evals}")
print("[PASS] All checks passed." if ok else "[FAIL] One or more checks failed.")
sys.exit(0 if ok else 1)
