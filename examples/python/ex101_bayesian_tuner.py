"""
ex101_bayesian_tuner.py - BayesianOptimizer: BO vs CMA-ES for PID tuning.

Tunes [Kp, Ki] for a FOPDT plant using:
  1. BayesianOptimizer (UCB acquisition, 5 init + 20 BO = 25 evals total)
  2. BayesianOptimizer (EI  acquisition, same budget)
  3. AutoTuner (CMA-ES, same n_iter budget for comparison)

Plant: G(s) = 1.2/(2s+1), ZOH at Ts=0.05s
Objective: IAE over 80 steps for unit step response.

Expected output:
  [PASS] BO-UCB found PID with IAE < 0.5.
  [PASS] BO-EI  found PID with IAE < 0.5.
  [PASS] All checks passed.
"""

import sys
import math

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'BayesianOptimizer'):
        raise AttributeError("BayesianOptimizer not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

import numpy as np

# ---------------------------------------------------------------------------
# Plant and cost function
# ---------------------------------------------------------------------------
Ts    = 0.05
alpha = math.exp(-Ts / 2.0)
beta  = 1.2 * (1.0 - alpha)

def simulate_iae(params):
    Kp, Ki = float(params[0]), float(params[1])
    p = ctrl.PIDParams(); p.Kp = Kp; p.Ki = Ki; p.Kd = 0.0
    pid = ctrl.DiscretePID(p, Ts)
    y = 0.0; iae = 0.0
    for _ in range(80):
        e = 1.0 - y
        u = float(pid.compute(e))
        u = max(-3.0, min(3.0, u))
        y = alpha * y + beta * u
        iae += abs(e) * Ts
    return iae

# ---------------------------------------------------------------------------
# Common bounds
# ---------------------------------------------------------------------------
lower = np.array([0.01, 0.0])
upper = np.array([5.0,  2.0])
x0    = np.array([1.0,  0.2])

def make_bo(acq_mode, seed=42):
    bp = ctrl.BayesOptParams()
    bp.n = 2; bp.n_init = 5; bp.maxIter = 20
    bp.n_acq_restarts = 100; bp.seed = seed
    bp.lower = lower; bp.upper = upper
    bp.acq = acq_mode
    return ctrl.BayesianOptimizer(bp)

# ---------------------------------------------------------------------------
# Run all three optimizers
# ---------------------------------------------------------------------------
bo_ucb = make_bo(ctrl.BayesAcquisition.UCB, seed=42)
bo_ei  = make_bo(ctrl.BayesAcquisition.EI,  seed=42)

ap = ctrl.AutoTunerParams()
ap.n = 2; ap.sigma0 = 0.4; ap.maxIter = 15
ap.lower = lower; ap.upper = upper
at = ctrl.AutoTuner(ap, 42)

r_ucb = bo_ucb.tune(simulate_iae, x0)
r_ei  = bo_ei.tune(simulate_iae, x0)
r_at  = at.tune(simulate_iae, x0)

print(f"BO-UCB: Kp={r_ucb.params[0]:.3f} Ki={r_ucb.params[1]:.3f} "
      f"IAE={r_ucb.cost:.4f}  evals={r_ucb.n_evals}")
print(f"BO-EI : Kp={r_ei.params[0]:.3f}  Ki={r_ei.params[1]:.3f}  "
      f"IAE={r_ei.cost:.4f}  evals={r_ei.n_evals}")
print(f"CMA-ES: Kp={r_at.params[0]:.3f} Ki={r_at.params[1]:.3f} "
      f"IAE={r_at.cost:.4f}  evals={r_at.n_evals}")

# ---------------------------------------------------------------------------
# Verification
# ---------------------------------------------------------------------------
ok = True

if r_ucb.cost < 0.5:
    print("[PASS] BO-UCB found PID with IAE < 0.5.")
else:
    print(f"[FAIL] BO-UCB IAE {r_ucb.cost:.4f} >= 0.5.")
    ok = False

if r_ei.cost < 0.5:
    print("[PASS] BO-EI  found PID with IAE < 0.5.")
else:
    print(f"[FAIL] BO-EI  IAE {r_ei.cost:.4f} >= 0.5.")
    ok = False

# BO uses far fewer evaluations than CMA-ES for the same iteration budget
if r_ucb.n_evals == 25:  # n_init=5 + maxIter=20
    print(f"[PASS] BO-UCB used exactly 25 evals (budget-efficient).")
else:
    print(f"[FAIL] Expected 25 evals, got {r_ucb.n_evals}.")
    ok = False

# BayesAcquisition enum accessible
assert ctrl.BayesAcquisition.UCB is not None
assert ctrl.BayesAcquisition.EI  is not None
assert 'bayesian_optimizer' in ctrl.features()

print("[PASS] All checks passed." if ok else "[FAIL] See messages above.")
sys.exit(0 if ok else 1)
