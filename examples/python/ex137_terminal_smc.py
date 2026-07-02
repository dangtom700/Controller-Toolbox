"""
ex137_terminal_smc.py

NonsingularTerminalSMC (finite-time SMC) regulating a discrete integrator
y[k+1] = y[k] + b*u[k].  An integrator holds any y at zero control, so an SMC
without integral action reaches the setpoint with no steady-state offset.

Mirrors ex121_terminal_smc.cpp.
"""

import sys
import _setup_bindings  # noqa: F401

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'NonsingularTerminalSMC'):
        raise AttributeError("NonsingularTerminalSMC not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

Ts, b, ref = 0.01, 0.1, 1.0

p = ctrl.NonsingularTerminalSMCParams()
p.beta, p.gamma = 1.0, 1.5
p.K, p.eta, p.phi = 2.0, 0.5, 0.5
p.uMin, p.uMax = -20.0, 20.0
smc = ctrl.NonsingularTerminalSMC(p, Ts)

y = 0.0
e0 = abs(ref - y)
reached = -1
for k in range(801):
    u = smc.compute(y - ref)          # SMC sign convention: y - ref
    y = y + b * u
    if reached < 0 and abs(smc.sliding_surface()) < p.phi:
        reached = k
    if k % 100 == 0:
        print(f"k={k:4d}  y={y:.4f}  s={smc.sliding_surface():.4f}  u={u:.4f}")

ef = abs(ref - y)
print(f"Initial |e|={e0:.4f}, final |e|={ef:.4f}, reached boundary layer at step {reached}")

ok = (y == y) and ef < 0.02 and reached >= 0
print("[PASS]" if ok else "[FAIL]")
sys.exit(0 if ok else 1)
