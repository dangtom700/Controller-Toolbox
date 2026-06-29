"""
ex134_event_triggered_wrapper.py

Aperiodic-sampling (event-triggered) control via EventTriggeredWrapper.

Mirrors ex117_event_triggered_wrapper.cpp -- wraps a PID so it only recomputes when the
tracking error drifts more than a deadband since the last triggered computation, holding the
output otherwise. Uses a slowly-varying (sinusoidal) reference rather than a step: once a step
response settles to a fixed point, the error stops drifting and can permanently fall under the
deadband, locking the wrapper out of ever re-triggering again - even though the settled error
may sit well outside the deadband. A continuously-varying reference keeps the error drifting
indefinitely, so the tracking-error bound below (checked over the back half of the run, past
the startup transient) is actually exercised throughout.
"""

import sys
import math
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
pp.Kp, pp.Ki, pp.Kd, pp.N = 4.0, 2.0, 0.0, 10.0
pid = ctrl.DiscretePID(pp, Ts)

etp = ctrl.EventTriggeredParams()
etp.sigma = 0.05
etw = ctrl.EventTriggeredWrapper(pid, etp)

x_plant = np.zeros(plant.state_size())
y = 0.0
ref_base, ref_amp, ref_period_s = 1.0, 0.3, 20.0
N = 300

max_err_tail = 0.0  # worst |ref - y| over the back half (transient excluded)
for k in range(N):
    t = k * Ts
    ref = ref_base + ref_amp * math.sin(2.0 * math.pi * t / ref_period_s)
    e = ref - y
    u = etw.compute(e)
    y_arr, x_plant = ctrl.ss_step_copy(plant, x_plant, np.array([u]))
    y = float(y_arr[0])
    if k >= N // 2:
        max_err_tail = max(max_err_tail, abs(ref - y))

print(f"Total compute() calls: {N}")
print(f"Triggered: {etw.trigger_count()}  Held: {etw.hold_count()}")
print(f"Max |error| (back half) = {max_err_tail:.6f}")

fewer_triggers_than_calls = etw.trigger_count() < N
tracks_well = bool(np.isfinite(max_err_tail)) and max_err_tail < 4.0 * etp.sigma
counts_consistent = (etw.trigger_count() + etw.hold_count()) == N

ok = fewer_triggers_than_calls and tracks_well and counts_consistent
print("[PASS]" if ok else "[FAIL]")
sys.exit(0 if ok else 1)
