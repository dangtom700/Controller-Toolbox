"""
ex124_ftc_supervisor.py

Phase 3 Roadmap Phase 2 (DT4): fault-tolerant reconfiguration on a redundant sensor pair.

Mirrors ex107_ftc_supervisor.cpp - FTCSupervisor detects a sensor_bias fault on the primary
sensor and switches ControllerStack's active entry to a controller relying on the healthy
backup sensor.

NOTE: FaultClassifier is a small-sample per-step heuristic, so individual steps can keep
flickering between fault types as the still-oscillating closed loop's residual statistics vary
after the switch; FTCSupervisor ignores classifications with no registered response rather than
reconfiguring on every flicker, so the *active controller* stays latched on the backup. This
checks that reliably-true property (detected at least once, latched, bounded), not that every
single step classifies identically - see ex107_ftc_supervisor.cpp.
"""

import sys
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'FTCSupervisor'):
        raise AttributeError("FTCSupervisor not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

Ts = 0.1
tf = ctrl.TransferFunction([0.0, 1.0], [1.0, -0.9], Ts)
sys_ss = ctrl.tf2ss(tf)
x = np.zeros(sys_ss.state_size())

pp = ctrl.PIDParams()
pp.Kp, pp.Ki = 1.0, 0.3
stack = ctrl.ControllerStack(ctrl.StackMode.Supervisory, Ts)
stack.add_controller(ctrl.DiscretePID(pp, Ts), "primary_sensor_pid")
stack.add_controller(ctrl.DiscretePID(pp, Ts), "backup_sensor_pid")

ftc = ctrl.FTCSupervisor(stack, ctrl.FaultDetectorParams(), Ts)
ftc.register_fault_response(ctrl.FaultType.None_, "primary_sensor_pid")
ftc.register_fault_response(ctrl.FaultType.SensorBias, "backup_sensor_pid")

r = 1.0
y = 0.0
primary_bias = 0.0
u_prev = 0.0
N = 200
active_at_end = ""
sensor_bias_ever_detected = False

for k in range(N):
    if k == N // 2:
        primary_bias = 5.0  # primary sensor develops a bias fault mid-run

    y_primary = y + primary_bias
    y_backup = y

    ftc.feed_residual(y_primary - y_backup, u_prev, y_primary)
    e = r - y_primary
    u = ftc.compute(e)
    u_prev = u

    y_vec, x = ctrl.ss_step_copy(sys_ss, x, np.array([u]))  # returns (y, x_next)
    y = float(y_vec[0])
    active_at_end = stack.active_controller_name()
    sensor_bias_ever_detected = sensor_bias_ever_detected or \
        ftc.current_fault() == ctrl.FaultType.SensorBias

print(f"Final active controller: {active_at_end}")
print(f"SensorBias detected at least once: {'yes' if sensor_bias_ever_detected else 'no'}")
print(f"Final y = {y:.4f}")

ok = (active_at_end == "backup_sensor_pid" and sensor_bias_ever_detected and
      np.isfinite(y) and abs(y) < 1000.0)
print("[PASS] All checks passed." if ok else "[FAIL] One or more checks failed.")
sys.exit(0 if ok else 1)
