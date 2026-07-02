"""
ex139_fractional_order_pid.py

FractionalOrderPID (PI^lambda D^mu) with Oustaloup-approximated operators.

Part 1: the standalone fractional operator s^0.5, band centred at 1 rad/s, has
        magnitude |s^0.5| = 1 at the centre.
Part 2: closed-loop step tracking on y[k+1] = 0.8*y + 0.2*u (DC gain 1); the
        fractional-integral branch supplies the steady control.

Mirrors ex123_fractional_order_pid.cpp.
"""

import sys
import math
import _setup_bindings  # noqa: F401

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'FractionalOrderPID') or not hasattr(ctrl, 'FractionalDifferintegrator'):
        raise AttributeError("FractionalOrderPID/FractionalDifferintegrator not found - rebuild binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

# ---- Part 1: operator magnitude at the band centre ----
Tsf = 0.005
half = ctrl.FractionalDifferintegrator(0.5, 0.01, 100.0, 5, Tsf)
w = 1.0
ymax = 0.0
Nsin = 40000
for k in range(Nsin):
    out = half.compute(math.sin(w * k * Tsf))
    if k > Nsin - 4000:
        ymax = max(ymax, abs(out))
print(f"Oustaloup s^0.5 gain at band centre = {ymax:.4f} (expected ~1.0)")
op_ok = (ymax == ymax) and 0.85 < ymax < 1.15

# ---- Part 2: closed-loop FOPID tracking ----
Ts = 0.01
p = ctrl.FOPIDParams()
p.Kp, p.Ki, p.Kd = 0.5, 0.3, 0.02
p.lam, p.mu = 0.9, 0.6          # 'lambda' is a Python keyword -> bound as 'lam'
p.wb, p.wh, p.N = 0.01, 100.0, 4
p.uMin, p.uMax = -10.0, 10.0
fopid = ctrl.FractionalOrderPID(p, Ts)

y = 0.0
ref = 1.0
for k in range(2001):
    u = fopid.compute(ref - y)
    y = 0.8 * y + 0.2 * u
    if k % 250 == 0:
        print(f"k={k:4d}  y={y:.4f}  u={u:.4f}")

ef = abs(ref - y)
print(f"s^0.5 gain OK={op_ok}, final |e|={ef:.4f}")

ok = op_ok and (y == y) and ef < 0.1
print("[PASS]" if ok else "[FAIL]")
sys.exit(0 if ok else 1)
