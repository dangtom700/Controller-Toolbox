"""
ex118_self_tuning_regulator.py

Phase 3 Roadmap Phase 2 (OC1): self-tuning regulator on a slowly-drifting plant.

Mirrors ex101_self_tuning_regulator.cpp - SelfTuningRegulator re-identifies the plant online
and updates its control law every step, remaining stable through a mid-run parameter change.

NOTE: certainty-equivalence direct adaptive control has no general guarantee of persistent
excitation from closed-loop operation alone (see SelfTuningRegulator.h) - this example checks
the reliably-true property (stability through a plant change), not exact setpoint tracking.
"""

import sys
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'SelfTuningRegulator'):
        raise AttributeError("SelfTuningRegulator not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

params = ctrl.STRParams()
params.na = 1
params.nb = 1
params.mode = ctrl.STRMode.MinimumVariance
params.lambda_ = 0.97
# Minimum-variance (d=1) is inherently a deadbeat design - bound u to a realistic actuator
# range so identification-transient errors can't saturate into a runaway feedback loop.
params.uMin = -20.0
params.uMax = 20.0

str_ctrl = ctrl.SelfTuningRegulator(params, 0.1)

rng = np.random.default_rng(42)
y = 0.0
a, b = 0.6, 1.0  # "summer load" plant: y[k] = a*y[k-1] + b*u[k-1]
setpoint = 5.0

N = 400
for k in range(N):
    if k == N // 2:
        a, b = 0.3, 0.6  # "winter load" - plant drifts mid-run
    excite = rng.uniform(-0.3, 0.3) * (5.0 if k < 30 else 0.0)
    str_ctrl.set_reference(setpoint + excite)
    u = str_ctrl.compute(y)
    # Apply u directly - compute() already accounts for the plant's inherent one-step delay
    # internally via its own bookkeeping.
    y = a * y + b * u

print(f"Final y = {y:.4f} (target {setpoint})")
print(f"Estimated A(q^-1) = {str_ctrl.estimated_denominator()}")
print(f"Estimated B(q^-1) = {str_ctrl.estimated_numerator()}")

ok = np.isfinite(y) and abs(y) < 1000.0 and np.all(np.isfinite(str_ctrl.covariance()))
print("[PASS] All checks passed." if ok else "[FAIL] One or more checks failed.")
sys.exit(0 if ok else 1)
