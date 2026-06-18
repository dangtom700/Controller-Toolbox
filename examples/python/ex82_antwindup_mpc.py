"""
ex82_antwindup_mpc.py
---------------------
Part 24: AntiWindupWrapper binding demonstration.

Wraps a DiscretePID (no built-in anti-windup) with AntiWindupWrapper and shows
that the conditioning technique prevents integrator windup on a saturating plant.

Plant:  y[k+1] = 0.9*y[k] + 0.2*u[k]  (first-order, y_ss=2 at u=1)
PI:     Kp=0.5, Ki=1.0, Ts=0.1         (Kb=0 - no built-in anti-windup)
Limits: uMin=-3, uMax=1
Phase 1 (50 steps): r=5  - actuator saturates throughout
Phase 2 (30 steps): r=0  - recovery; wrapped version unwinds much faster

Acceptance:
  - Wrapped y_final < 1.5   (converged toward r=0 by end of recovery)
  - Unwrapped y_final > 1.5 (still winding down near y_ss=2)

Expected output: PASS
"""
import sys
import numpy as np

sys.path.insert(0, __file__.rsplit("\\", 1)[0])
import _setup_bindings
try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'AntiWindupWrapper'):
        raise AttributeError("AntiWindupWrapper not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

Ts   = 0.1
uMin = -3.0
uMax =  1.0
Nsat = 50
Nrec = 30


def step_plant(y, u):
    return 0.9 * y + 0.2 * u


def make_pid():
    pp = ctrl.PIDParams()
    pp.Kp = 0.5; pp.Ki = 1.0; pp.Kd = 0.0
    pp.N = 10.0; pp.Kb = 0.0          # no internal anti-windup
    pp.uMin = -1e9; pp.uMax = 1e9     # wrapper owns the clamping
    return ctrl.DiscretePID(pp, Ts)


# --- Unwrapped: manual output clamp, integral winds up freely ---
pid_raw = make_pid()
y_raw = 0.0
for k in range(Nsat + Nrec):
    ref = 5.0 if k < Nsat else 0.0
    u   = float(np.clip(pid_raw.compute(ref - y_raw), uMin, uMax))
    y_raw = step_plant(y_raw, u)

# --- Wrapped: conditioning technique prevents windup ---
pid_inner = make_pid()
wrapped = ctrl.AntiWindupWrapper(pid_inner, uMin, uMax, 1.0)
y_aw = 0.0
for k in range(Nsat + Nrec):
    ref = 5.0 if k < Nsat else 0.0
    u   = wrapped.compute(ref - y_aw)
    y_aw = step_plant(y_aw, u)

print(f"Unwrapped y_final = {y_raw:.4f}  (integral wound up)")
print(f"Wrapped   y_final = {y_aw:.4f}  (conditioning bounded integral)")

pass_aw   = y_aw  < 1.5
pass_raw  = y_raw > 1.5
pass_gap  = y_aw  < y_raw

if pass_aw and pass_raw and pass_gap:
    print("PASS")
else:
    if not pass_aw:
        print(f"FAIL: wrapped y={y_aw:.4f} not < 1.5 (anti-windup not effective)")
    if not pass_raw:
        print(f"FAIL: unwrapped y={y_raw:.4f} not > 1.5 (expected windup not present)")
    if not pass_gap:
        print(f"FAIL: wrapped y={y_aw:.4f} not < unwrapped y={y_raw:.4f}")
    sys.exit(1)
