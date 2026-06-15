"""
ex95_registry_monitor.py - M2/M3: ControllerRegistry + ControllerMonitor SPC.

Demonstrates:
  1. M2 - ctrl.features() now delegates to the self-registration registry.
     Shows dyna, scenario_mpc, bayesian_optimizer all appear automatically.
  2. M3 - ControllerMonitor attached to a DiscreteSMC; CUSUM chart detects
     a sustained mean shift in the sliding surface after a disturbance.
  3. IControllerObserver.on_state() override in Python works correctly.

Expected output:
  [PASS] Registry has >= 30 features.
  [PASS] Self-registered features present.
  [PASS] CUSUM alarm fires on surface shift.
  [PASS] All checks passed.
"""

import sys, os
import _setup_bindings  # noqa: F401

import math

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'ControllerMonitor'):
        raise AttributeError("ControllerMonitor not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

import numpy as np

ok = True

# ---------------------------------------------------------------------------
# M2: registry
# ---------------------------------------------------------------------------
feat = ctrl.features()
print(f"Registered features: {len(feat)}")
if len(feat) >= 30:
    print("[PASS] Registry has >= 30 features.")
else:
    print(f"[FAIL] Expected >= 30, got {len(feat)}")
    ok = False

for expected in ['pid', 'dyna', 'scenario_mpc', 'bayesian_optimizer', 'controller_monitor']:
    if not ctrl.registry_has(expected):
        print(f"[FAIL] '{expected}' not in registry")
        ok = False

if ok:
    print("[PASS] Self-registered features present.")

# ---------------------------------------------------------------------------
# M3: Python observer subclass catching on_state
# ---------------------------------------------------------------------------
class StateCapture(ctrl.IControllerObserver):
    def __init__(self):
        super().__init__()
        self.surface_vals = []

    def on_state(self, key, value):
        if key == "surface":
            self.surface_vals.append(float(value[0]))

Ts    = 0.05
alpha = math.exp(-Ts / 1.0)
beta  = 1.0 - alpha

smc_p = ctrl.SMCParams()
smc_p.c_e = 1.0; smc_p.c_de = 0.0; smc_p.K = 2.0; smc_p.phi = 0.1
smc_p.uMin = -5.0; smc_p.uMax = 5.0
smc = ctrl.DiscreteSMC(smc_p, Ts)

cap = StateCapture()
smc.attach_observer(cap)

y = 0.0
for k in range(50):
    e = 1.0 - y
    u = smc.compute(e)
    y = alpha * y + beta * u

if len(cap.surface_vals) == 50:
    print("[PASS] on_state received surface values for all 50 steps.")
else:
    print(f"[FAIL] Expected 50 surface readings, got {len(cap.surface_vals)}")
    ok = False

# ---------------------------------------------------------------------------
# ControllerMonitor: CUSUM alarm on mean-shifted output
# ---------------------------------------------------------------------------
mon = ctrl.ControllerMonitor()
mon.set_target(0.0); mon.set_sigma(0.1)
mon.set_cusum_params(0.5, 4.0)
mon.set_ewma_params(0.2, 3.0)

alarms = []
mon.set_alarm_callback(lambda chart, stat: alarms.append((chart, stat)))

# In-control: feed 10 zero samples
for _ in range(10):
    mon.on_compute(0.0, 0.0)

# Out-of-control: feed 20 samples with +2 shift
for _ in range(20):
    mon.on_compute(2.0, 0.0)

if alarms:
    print(f"[PASS] CUSUM alarm fires on surface shift. First: {alarms[0]}")
else:
    print("[FAIL] No alarm after sustained mean shift.")
    ok = False

if mon.n_samples() == 30:
    print("[PASS] ControllerMonitor tracked all 30 samples.")
else:
    print(f"[FAIL] Expected 30 samples, got {mon.n_samples()}")
    ok = False

print("[PASS] All checks passed." if ok else "[FAIL] See messages above.")
sys.exit(0 if ok else 1)
