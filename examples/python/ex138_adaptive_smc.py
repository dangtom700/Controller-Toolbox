"""
ex138_adaptive_smc.py

AdaptiveSMC rejecting a matched disturbance of UNKNOWN bound on an integrator
y[k+1] = y[k] + b*(u[k] + d).  The switching gain K adapts online (grows from a
deliberately-too-small K0), so no a-priori disturbance bound is required.

Mirrors ex122_adaptive_smc.cpp.
"""

import sys
import _setup_bindings  # noqa: F401

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'AdaptiveSMC'):
        raise AttributeError("AdaptiveSMC not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

Ts, b, d, ref = 0.01, 0.1, 0.3, 1.0

p = ctrl.AdaptiveSMCParams()
p.c_e, p.c_de = 1.0, 0.05
p.gamma, p.epsilon = 8.0, 0.02
p.K0, p.Kmin, p.Kmax = 0.2, 0.0, 100.0
p.phi = 0.3
p.uMin, p.uMax = -20.0, 20.0
smc = ctrl.AdaptiveSMC(p, Ts)

y = 0.0
K_start = smc.adaptive_gain()
for k in range(1501):
    u = smc.compute(y - ref)
    y = y + b * (u + d)
    if k % 150 == 0:
        print(f"k={k:4d}  y={y:.4f}  K={smc.adaptive_gain():.4f}  u={u:.4f}")

ef = abs(ref - y)
K_final = smc.adaptive_gain()
print(f"Gain adapted {K_start:.4f} -> {K_final:.4f}, final |e|={ef:.4f}")

ok = (y == y) and ef < 0.05 and K_final > K_start
print("[PASS]" if ok else "[FAIL]")
sys.exit(0 if ok else 1)
