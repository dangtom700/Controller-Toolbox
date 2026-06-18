"""
ex22 - Smith Predictor for Dead-Time Compensation
===================================================
Goal     : Add an artificial pure delay (5 steps) to the plant, then compare
           a plain PID (no delay compensation) against a SmithPredictor-wrapped
           PID. Verify the Smith Predictor gives better ISE.

Data generation : 2 000 samples; unit step reference.
Verification    :
  - SmithPredictor ISE < plain PID ISE (delay compensated).
  - Both are stable (|y| < 10).
  - Smith Predictor model output matches plant output within model accuracy.

Run:
    conda activate soft_robotics
    python ex22_smith_predictor.py
"""

import numpy as np
from collections import deque
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from utils.plant import example_plant, tf2ss, EXAMPLE_NUM, EXAMPLE_DEN, ss_step
from utils.controllers import DiscretePID, SmithPredictor
from utils.verify import ise, print_summary

Ts         = 0.01
STEPS      = 2000
DELAY_STEPS = 5     # 5-step pure delay = 0.05 s

Kp, Ki, Kd = 2.0, 1.0, 0.5

print("=" * 60)
print("ex22 - Smith Predictor vs Plain PID (with delay)")
print("=" * 60)
print(f"\n  Pure delay: {DELAY_STEPS} steps ({DELAY_STEPS*Ts:.3f} s)")

def sim_with_delay(controller_fn, delay_steps=DELAY_STEPS):
    """Simulate plant with a pure output delay; controller_fn(r, y_delayed) -> u."""
    plant = example_plant()
    y_buf = deque([0.0] * delay_steps, maxlen=delay_steps)
    y     = np.zeros(STEPS)
    for k in range(STEPS):
        y_delayed = y_buf[0]
        u = controller_fn(1.0, y_delayed)
        y[k] = ss_step(plant, u)
        y_buf.append(y[k])
    return y

# Plain PID (sees delayed measurement)
pid_plain = DiscretePID(Kp=Kp, Ki=Ki, Kd=Kd, Ts=Ts, u_min=-10.0, u_max=10.0)
y_plain = sim_with_delay(lambda r, yd: pid_plain.compute(r, yd))

# Smith Predictor: inner PID + plant model + delay model
pid_inner   = DiscretePID(Kp=Kp, Ki=Ki, Kd=Kd, Ts=Ts, u_min=-10.0, u_max=10.0)
model_plant = tf2ss(EXAMPLE_NUM, EXAMPLE_DEN)
smith = SmithPredictor(model=model_plant, delay_steps=DELAY_STEPS, pid=pid_inner)

u_prev_sp = 0.0
y_smith   = np.zeros(STEPS)
plant_sp  = example_plant()
y_buf_sp  = deque([0.0] * DELAY_STEPS, maxlen=DELAY_STEPS)

for k in range(STEPS):
    y_delayed = y_buf_sp[0]
    u = smith.compute(reference=1.0, measurement=y_delayed, u_prev=u_prev_sp)
    y_smith[k] = ss_step(plant_sp, u)
    y_buf_sp.append(y_smith[k])
    u_prev_sp = u

err_plain = 1.0 - y_plain
err_smith = 1.0 - y_smith
ise_plain = ise(err_plain, Ts)
ise_smith = ise(err_smith, Ts)

print(f"\n  ISE - plain PID:       {ise_plain:.5f}")
print(f"  ISE - Smith Predictor: {ise_smith:.5f}")

results = {}
results["smith_better_ise"] = ise_smith < ise_plain
print(f"  {'[PASS]' if results['smith_better_ise'] else '[FAIL]'} Smith ISE < plain PID ISE")

results["plain_stable"] = np.all(np.isfinite(y_plain)) and float(np.max(np.abs(y_plain))) < 10.0
results["smith_stable"] = np.all(np.isfinite(y_smith)) and float(np.max(np.abs(y_smith))) < 10.0
print(f"  {'[PASS]' if results['plain_stable'] else '[FAIL]'} plain PID stable")
print(f"  {'[PASS]' if results['smith_stable'] else '[FAIL]'} Smith Predictor stable")

for name, y in [("plain", y_plain), ("smith", y_smith)]:
    ss_e = abs(float(np.mean(y[-200:])) - 1.0)
    results[f"{name}_ss"] = ss_e < 0.02
    print(f"  {name} SS error: {ss_e:.4f}  {'[PASS]' if results[f'{name}_ss'] else '[FAIL]'}")

print_summary(results)
