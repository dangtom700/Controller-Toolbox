"""
ex94_cbf_safety.py -- Control Barrier Function (CBF) safety filter demo.

Demonstrates:
  1. A PID controller trying to drive an integrator to r=2.0 (above safe limit).
  2. CBF filter keeping the state below x_max=1.5 at all times.
  3. Reporting when the CBF constraint is active vs. inactive.

Plant:  x[k+1] = x[k] + Ts*u[k]   (pure integrator)
Safe set:  h(x) = x_max - x >= 0,  x_max = 1.5
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'build', 'bindings'))
if sys.platform == "win32" and hasattr(os, "add_dll_directory"):
    for _p in [r"C:\msys64\mingw64\bin"]:
        if os.path.isdir(_p):
            os.add_dll_directory(_p)

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'CBFSafetyFilter'):
        raise AttributeError("CBFSafetyFilter not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

import numpy as np

print("=== ex94: CBF Safety Filter ===")

TS    = 0.01
X_MAX = 1.5
N     = 400
R     = 2.0   # desired reference - above the safe limit!

# ---------------------------------------------------------------------------
# Nominal PID (aggressive - would violate safety without CBF)
# ---------------------------------------------------------------------------
pid = ctrl.DiscretePID(8.0, 2.0, 0.0, TS, 1.0)

# ---------------------------------------------------------------------------
# CBF safety filter
#   Safe set: h(x) = x_max - x >= 0
#   Plant:    x_dot = u  (pure integrator => f0=0, g=1)
# ---------------------------------------------------------------------------
cbf_p = ctrl.CBFParams()
cbf_p.alpha = 3.0
cbf_p.uMin  = -5.0
cbf_p.uMax  =  5.0

cbf = ctrl.CBFSafetyFilter(
    pid,
    lambda x: X_MAX - x,   # h(x)
    lambda x: -1.0,         # dh/dx
    lambda x: 0.0,          # f0(x): no drift
    lambda x: 1.0,          # g(x): input gain
    cbf_p,
    TS
)

# ---------------------------------------------------------------------------
# Closed-loop simulation
# ---------------------------------------------------------------------------
x = 0.0
x_max_seen  = 0.0
cbf_active_count = 0

print(f"\n  {'k':>5}  {'x':>8}  {'u_nom':>8}  {'u_safe':>8}  {'cbf?':>6}  {'h(x)':>8}")

for k in range(N):
    cbf.set_state(x)
    u_safe = cbf.compute(R - x)

    if cbf.cbf_active():
        cbf_active_count += 1

    if k in (0, 50, 100, 150, 200, 300, 399):
        print(f"  {k:5d}  {x:8.4f}  {cbf.nominal_output():8.4f}"
              f"  {u_safe:8.4f}  {'YES':>6}  {cbf.barrier_value():8.4f}"
              if cbf.cbf_active() else
              f"  {k:5d}  {x:8.4f}  {cbf.nominal_output():8.4f}"
              f"  {u_safe:8.4f}  {'no':>6}  {cbf.barrier_value():8.4f}")

    x += TS * u_safe
    x_max_seen = max(x_max_seen, x)

print(f"\n  Max x reached:    {x_max_seen:.4f}  (limit: {X_MAX})")
print(f"  CBF active steps: {cbf_active_count} / {N}")
print(f"  Final x:          {x:.4f}")

assert x_max_seen <= X_MAX + 0.01, f"Safety violated: x_max={x_max_seen:.4f} > {X_MAX}"
print("\n[PASS] CBFSafetyFilter demo complete - safety constraint respected.")
