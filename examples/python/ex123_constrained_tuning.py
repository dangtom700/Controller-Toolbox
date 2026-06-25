"""
ex123_constrained_tuning.py

Phase 3 Roadmap Phase 2 (MO3): tuning a PID subject to a closed-loop pole constraint.

Mirrors ex106_constrained_tuning.cpp - tune_constrained wraps AutoTuner's CMA-ES search with a
general nonlinear constraint (closed-loop dominant pole magnitude) via an exterior penalty.
"""

import sys
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'tune_constrained'):
        raise AttributeError("tune_constrained not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

Ts = 0.05
plant_tf = ctrl.TransferFunction([0.0, 1.0], [1.0, -0.9], Ts)


def tracking_cost(gains):
    pp = ctrl.PIDParams()
    pp.Kp, pp.Ki, pp.Kd = float(gains[0]), float(gains[1]), 0.0
    pid = ctrl.DiscretePID(pp, Ts)
    sys_ss = ctrl.tf2ss(plant_tf)
    x = np.zeros(sys_ss.state_size())

    cost, y = 0.0, 0.0
    for _ in range(100):
        e = 1.0 - y
        u = pid.compute(e)
        y_vec, x = ctrl.ss_step_copy(sys_ss, x, np.array([u]))  # returns (y, x_next)
        y = float(y_vec[0])
        cost += e * e
    return cost


def closed_loop_pole_magnitude(gains):
    a = plant_tf.den[1]
    b = plant_tf.num[1]
    Kp, Ki = float(gains[0]), float(gains[1])

    Acl = np.array([
        [-a - b * Kp, b],
        [-Ki * Ts * (-a - b * Kp), 1.0 - Ki * Ts * b],
    ])
    return float(np.max(np.abs(np.linalg.eigvals(Acl))))


pole_limit = 0.85
cp = ctrl.ConstrainedTuneParams()
cp.constraints = lambda gains: np.array([closed_loop_pole_magnitude(gains) - pole_limit])
cp.outer_iters = 6

atp = ctrl.AutoTunerParams()
atp.n = 2
atp.lower = np.array([0.0, 0.0])
atp.upper = np.array([10.0, 10.0])
tuner = ctrl.AutoTuner(atp)

result = ctrl.tune_constrained(
    lambda cost, x0: tuner.tune(cost, x0),
    tracking_cost, cp, np.array([1.0, 0.5]))

final_pole = closed_loop_pole_magnitude(result.params)
print(f"Tuned gains: Kp={result.params[0]:.3f} Ki={result.params[1]:.3f}")
print(f"Closed-loop dominant pole magnitude: {final_pole:.3f} (limit {pole_limit})")
print(f"Tracking cost: {result.cost:.4f}")

ok = np.isfinite(result.cost) and final_pole <= pole_limit + 0.05
print("[PASS] All checks passed." if ok else "[FAIL] One or more checks failed.")
sys.exit(0 if ok else 1)
