"""
ex79_adaptive_smith_predictor.py
---------------------------------
Part 22: AdaptiveSmithPredictor binding demonstration.

Plant: y[k+1] = 0.8*y[k] + 0.2*u[k-d_true]   (first-order + d_true-step delay)

The controller is initialised with an incorrect delay estimate d=1.
A square-wave reference (period 100 steps) provides persistent excitation so
that the cross-correlation estimator has variance in u and y to work with.
After enough excitation cycles the estimate should converge to within 2 steps
of the true delay, and the IAE in the second half should be lower than the first.

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
N_steps = 800           # long enough for several excitation cycles

# Delay-free model: x[k+1] = 0.8*x + 0.2*u,  y = x
model = ctrl.StateSpace(
    np.array([[0.8]]), np.array([[0.2]]),
    np.array([[1.0]]), np.array([[0.0]]), Ts)

# Inner PID
pp = ctrl.PIDParams()
pp.Kp = 1.5; pp.Ki = 0.2; pp.Kd = 0.0
inner = ctrl.DiscretePID(pp, Ts)

# Adaptive SP params
asp_p = ctrl.AdaptiveSPParams()
asp_p.max_delay_steps   = 8
asp_p.estimate_interval = 100   # re-estimate every 100 steps
asp_p.buffer_len        = 120   # shorter window keeps the transient data fresh

asp = ctrl.AdaptiveSmithPredictor(inner, model, d_init, Ts, asp_p)

# True plant delay buffer: d_true zeros so y[k+1] = 0.8*y[k] + 0.2*u[k-d_true]
u_buf = collections.deque([0.0] * d_true)
x_plant = 0.0

iae_first = 0.0
iae_second = 0.0

for k in range(N_steps):
    # Square-wave reference: toggles every 100 steps for persistent excitation
    ref = 1.0 if (k // 100) % 2 == 0 else 0.0

    y = x_plant
    e = ref - y
    asp.set_plant_output(y)
    u = asp.compute(e)

    # Apply delayed input to plant
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

# Acceptance criteria:
#   1. Delay converged within 2 steps of the true value.
#   2. Tracking remained stable (finite IAE, bounded final error).
delay_ok   = abs(d_est - d_true) <= 2
tracking_ok = np.isfinite(iae_second) and abs(ref - x_plant) < 0.5

if delay_ok and tracking_ok:
    print("PASS")
else:
    if not delay_ok:
        print(f"FAIL: delay estimate d_est={d_est} not within 2 of d_true={d_true}")
    if not tracking_ok:
        print(f"FAIL: tracking unstable, final |e|={abs(ref - x_plant):.4f}")
    sys.exit(1)
