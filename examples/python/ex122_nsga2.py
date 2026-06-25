"""
ex122_nsga2.py

Phase 3 Roadmap Phase 2 (MO1): NSGA-II tuning a PID for settling time vs control effort.

Mirrors ex105_nsga2.cpp - returns the actual tradeoff curve instead of a single weighted-sum
compromise.
"""

import sys
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'NSGA2'):
        raise AttributeError("NSGA2 not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

Ts = 0.05
tf = ctrl.TransferFunction([0.0, 1.0], [1.0, -0.9], Ts)
sys_ss = ctrl.tf2ss(tf)


def simulate(gains):
    pp = ctrl.PIDParams()
    pp.Kp, pp.Ki, pp.Kd = float(gains[0]), float(gains[1]), 0.0
    pid = ctrl.DiscretePID(pp, Ts)

    x = np.zeros(sys_ss.state_size())
    y = 0.0
    err_energy, effort_energy = 0.0, 0.0
    for _ in range(100):
        e = 1.0 - y
        u = pid.compute(e)
        y_vec, x = ctrl.ss_step_copy(sys_ss, x, np.array([u]))  # returns (y, x_next)
        y = float(y_vec[0])
        err_energy += e * e
        effort_energy += u * u
    return np.array([err_energy, effort_energy])


params = ctrl.NSGA2Params()
params.n_dim, params.n_objectives = 2, 2
params.population, params.max_gen = 40, 40
params.lower = np.array([0.1, 0.0])
params.upper = np.array([5.0, 5.0])

nsga = ctrl.NSGA2(params)
result = nsga.optimize(simulate)

print(f"Pareto front ({result.front_params.shape[0]} points):")
for i in range(min(5, result.front_params.shape[0])):
    print(f"  Kp={result.front_params[i, 0]:.3f} Ki={result.front_params[i, 1]:.3f}  "
          f"-> errEnergy={result.front_objectives[i, 0]:.3f} "
          f"effortEnergy={result.front_objectives[i, 1]:.3f}")

ok = result.front_params.shape[0] >= 1 and np.all(np.isfinite(result.front_objectives))
print("[PASS] All checks passed." if ok else "[FAIL] One or more checks failed.")
sys.exit(0 if ok else 1)
