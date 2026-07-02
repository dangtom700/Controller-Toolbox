"""
ex140_super_twisting_smc.py

SuperTwistingSMC (2nd-order sliding mode) step tracking on
y[k+1] = 0.8*y + 0.2*u.  This controller already existed in lib/ but had no
Python binding / example until now - this closes that coverage gap.

Mirrors ex124_super_twisting_smc.cpp.
"""

import sys
import _setup_bindings  # noqa: F401

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'SuperTwistingSMC'):
        raise AttributeError("SuperTwistingSMC not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

Ts, ref = 0.01, 1.0

p = ctrl.SuperTwistingParams()
p.c_e, p.c_de = 1.0, 0.01
p.K1, p.K2 = 2.0, 3.0        # K2 > K1^2/4 = 1
p.uMin, p.uMax = -20.0, 20.0
smc = ctrl.SuperTwistingSMC(p, Ts)

# Discrete super-twisting settles into a small period-2 ripple; judge tracking by the
# MEAN output over a final window (averages the ripple out) rather than a single sample.
y = 0.0
e0 = abs(ref - y)
N, WIN = 1000, 200
y_sum, y_cnt = 0.0, 0
for k in range(N + 1):
    u = smc.compute(y - ref)     # SMC sign convention: y - ref
    y = 0.8 * y + 0.2 * u
    if k > N - WIN:
        y_sum += y
        y_cnt += 1
    if k % 100 == 0:
        print(f"k={k:4d}  y={y:.4f}  s={smc.sliding_surface():.4f}  u={u:.4f}")

y_mean = y_sum / y_cnt
ef = abs(ref - y_mean)
print(f"Initial |e|={e0:.4f}, mean-tracking |e|={ef:.4f} (over last {WIN} steps)")

ok = (y == y) and ef < 0.03
print("[PASS]" if ok else "[FAIL]")
sys.exit(0 if ok else 1)
