"""
ex79_adaptive_smith_predictor.py
---------------------------------
Part 22: AdaptiveSmithPredictor binding demonstration.

Plant: y[k] = 0.8*y[k-1] + 0.2*u[k-d_true]  (first-order + delay d_true=4)

The controller is initialised with an incorrect delay estimate d=1.
After 400 steps the cross-correlation should push the estimate towards
the true delay and IAE should decrease compared to the wrong-delay period.

Expected output: PASS
"""
import sys
import collections
import numpy as np

sys.path.insert(0, __file__.rsplit("\\", 1)[0])
import _setup_bindings
try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'AdaptiveSmithPredictor'):
        raise AttributeError("AdaptiveSmithPredictor not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

Ts = 0.1
d_true = 4
d_init = 1
N_steps = 500
ref = 1.0

# Delay-free model: 1D, A=0.8, B=0.2
model = ctrl.StateSpace(
    np.array([[0.8]]), np.array([[0.2]]),
    np.array([[1.0]]), np.array([[0.0]]), Ts)

# Inner PID
pp = ctrl.PIDParams()
pp.Kp = 1.5; pp.Ki = 0.2; pp.Kd = 0.0
inner = ctrl.DiscretePID(pp, Ts)

# Adaptive SP params
asp_p = ctrl.AdaptiveSPParams()
asp_p.max_delay_steps    = 8
asp_p.estimate_interval  = 120
asp_p.buffer_len         = 200

asp = ctrl.AdaptiveSmithPredictor(inner, model, d_init, Ts, asp_p)

# True plant simulation
u_buf = collections.deque([0.0] * (d_true + 1))
x_plant = 0.0

iae_first = 0.0
iae_second = 0.0

for k in range(N_steps):
    y = x_plant
    e = ref - y
    asp.set_plant_output(y)
    u = asp.compute(e)

    u_buf.append(u)
    u_del = u_buf.popleft()
    x_plant = 0.8 * x_plant + 0.2 * u_del

    if k < N_steps // 2:
        iae_first  += abs(e) * Ts
    else:
        iae_second += abs(e) * Ts

d_est = asp.estimated_delay_steps()
print(f"Estimated delay: {d_est}  (true: {d_true})")
print(f"IAE first half:  {iae_first:.4f}")
print(f"IAE second half: {iae_second:.4f}")
print(f"Final |e|: {abs(ref - x_plant):.4f}")

# Soft acceptance: delay within 2 of truth AND tracking is finite
delay_reasonable = abs(d_est - d_true) <= 2
tracking_ok = abs(ref - x_plant) < 0.5 and np.isfinite(iae_second)

if delay_reasonable and tracking_ok:
    print("PASS")
else:
    print(f"FAIL: d_est={d_est}, |e|={abs(ref - x_plant):.4f}")
    sys.exit(1)
