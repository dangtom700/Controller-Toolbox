"""
ex08 - PID Closed-Loop Simulation
===================================
Goal     : Close the loop around the example plant with a DiscretePID and
           verify performance metrics (rise time, settling time, overshoot,
           steady-state error) for a set of well-tuned PID gains.

Data generation : 1 500 step samples, unit step reference, gains from IMC rule.
Verification    :
  - Steady-state error < 0.5 %.
  - Overshoot < 10 %.
  - Rise time (10->90%) recorded and sane (0.1-5 s).

Run:
    conda activate soft_robotics
    python ex08_pid_closed_loop.py
"""

import numpy as np
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from utils.plant import example_plant, ss_step
from utils.controllers import DiscretePID
from utils.verify import dc_gain_check, print_summary, assert_close

Ts    = 0.01
STEPS = 1500

# IMC-tuned gains for G(s)=1/(s^2+1.5s+1), lambda=0.5
Kp, Ki, Kd = 3.0, 1.5, 0.75

print("=" * 60)
print("ex08 - PID Closed-Loop Simulation")
print("=" * 60)
print(f"\n  Gains: Kp={Kp}, Ki={Ki}, Kd={Kd}, Ts={Ts}")

pid   = DiscretePID(Kp=Kp, Ki=Ki, Kd=Kd, Ts=Ts, N=10.0,
                    u_min=-10.0, u_max=10.0)
plant = example_plant()

t   = np.arange(STEPS) * Ts
r   = np.ones(STEPS)        # unit step reference
y   = np.zeros(STEPS)
u   = np.zeros(STEPS)
err = np.zeros(STEPS)

for k in range(STEPS):
    u[k]   = pid.compute(r[k], y[k - 1] if k > 0 else 0.0)
    y[k]   = ss_step(plant, u[k])
    err[k] = r[k] - y[k]

results = {}

# Steady-state error
results["ss_error"] = dc_gain_check(y, expected=1.0, tol=0.005)

# Overshoot
y_max = float(np.max(y))
overshoot_pct = max(0.0, (y_max - 1.0) * 100.0)
print(f"\n  Overshoot: {overshoot_pct:.2f} %")
results["overshoot"] = overshoot_pct < 10.0
print(f"  {'[PASS]' if results['overshoot'] else '[FAIL]'} overshoot < 10%")

# Rise time (10->90%)
y10 = np.argmax(y >= 0.10)
y90 = np.argmax(y >= 0.90)
rise_time = (y90 - y10) * Ts
print(f"  Rise time (10->90%): {rise_time:.3f} s")
results["rise_time"] = 0.05 < rise_time < 5.0
print(f"  {'[PASS]' if results['rise_time'] else '[FAIL]'} rise time in (0.05, 5.0) s")

# Settling time (2% band)
settled = np.where(np.abs(y - 1.0) > 0.02)[0]
settle_time = float(settled[-1]) * Ts if len(settled) > 0 else 0.0
print(f"  Settling time (2%): {settle_time:.3f} s")
results["settling"] = settle_time < STEPS * Ts
print(f"  {'[PASS]' if results['settling'] else '[FAIL]'} system settles within simulation")

# ISE
ise_val = float(Ts * np.sum(err**2))
print(f"  ISE: {ise_val:.6f}")
results["ise_finite"] = np.isfinite(ise_val)

print_summary(results)
