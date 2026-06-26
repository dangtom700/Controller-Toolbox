"""
ex134_event_triggered_wrapper.py

Aperiodic-sampling (event-triggered) control via EventTriggeredWrapper.

Mirrors ex117_event_triggered_wrapper.cpp -- wraps a PID so it only recomputes when the
tracking error drifts more than a deadband since the last triggered computation, holding the
output otherwise.
"""

import sys
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'EventTriggeredWrapper'):
        raise AttributeError("EventTriggeredWrapper not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

Ts = 0.1

plant_c = ctrl.StateSpace(np.array([[-1.0]]), np.array([[1.0]]),
                           np.array([[1.0]]), np.array([[0.0]]), 0.0)
plant = ctrl.c2d(plant_c, Ts, ctrl.C2dMethod.ZOH)

pp = ctrl.PIDParams()
pp.Kp, pp.Ki, pp.Kd, pp.N = 2.0, 0.8, 0.0, 10.0
pid = ctrl.DiscretePID(pp, Ts)

etp = ctrl.EventTriggeredParams()
etp.sigma = 0.05
etw = ctrl.EventTriggeredWrapper(pid, etp)

x_plant = np.zeros(plant.state_size())
y = 0.0
ref = 1.0
N = 300

for _ in range(N):
    e = ref - y
    u = etw.compute(e)
    y, x_plant = ctrl.ss_step_copy(plant, x_plant, np.array([u]))

print(f"Total compute() calls: {N}")
print(f"Triggered: {etw.trigger_count()}  Held: {etw.hold_count()}")
print(f"Final y = {y:.4f} (reference = {ref})")

fewer_triggers_than_calls = etw.trigger_count() < N
tracks_well = bool(np.isfinite(y)) and abs(float(y) - ref) < 4.0 * etp.sigma
counts_consistent = (etw.trigger_count() + etw.hold_count()) == N

ok = fewer_triggers_than_calls and tracks_well and counts_consistent
print("[PASS]" if ok else "[FAIL]")
sys.exit(0 if ok else 1)
