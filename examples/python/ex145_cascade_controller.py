"""
ex145_cascade_controller.py -- CascadeController: series inner/outer composition.

Demonstrates:
  1. Outer PID output becomes the inner PID's setpoint (a SERIES hand-off, which
     ControllerStack.Additive cannot express - it sums outputs in parallel).
  2. A load disturbance entering the INNER loop is rejected before it propagates
     through the slow outer lag; a single outer PID only sees it after the delay.
  3. spMin/spMax clamping of the inner setpoint, with the outer loop back-calculated
     so its integrator does not wind up against a limit the inner loop cannot deliver.

Plant:  inner  x1[k+1] = a1*x1 + (1-a1)*(u + d)    tau = 0.5 s  (fast, disturbed)
        outer  x2[k+1] = a2*x2 + (1-a2)*x1         tau = 2.0 s  (slow, measured)

Sign convention: compute(r_outer - y_outer)  (same as DiscretePID).
"""

import sys
import _setup_bindings  # noqa: F401

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'CascadeController'):
        raise AttributeError("CascadeController not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

import numpy as np

print("=== ex145: CascadeController (series inner/outer) ===")

TS = 0.05
N = 1200
K_DIST = 400
D_MAG = 0.6
REF = 1.0

A1 = np.exp(-TS / 0.5)
A2 = np.exp(-TS / 2.0)


def outer_gains():
    p = ctrl.PIDParams()
    p.Kp, p.Ki, p.Kd = 1.2, 0.35, 0.0
    p.uMin, p.uMax = -5.0, 5.0
    return p


def inner_gains():
    p = ctrl.PIDParams()
    p.Kp, p.Ki, p.Kd = 2.5, 5.0, 0.0
    p.uMin, p.uMax = -10.0, 10.0
    return p


def simulate(controller, cascaded):
    """Run the two-lag plant; returns (IAE after the disturbance step, final y)."""
    x1 = x2 = iae = 0.0
    for k in range(N):
        d = D_MAG if k >= K_DIST else 0.0
        if cascaded:
            controller.set_inner_measurement(x1)
        u = controller.compute(REF - x2)
        if k >= K_DIST:
            iae += abs(REF - x2) * TS
        x2 = A2 * x2 + (1.0 - A2) * x1
        x1 = A1 * x1 + (1.0 - A1) * (u + d)
    return iae, x2


# ---------------------------------------------------------------------------
# 1. Single-loop baseline vs cascade, same outer gains
# ---------------------------------------------------------------------------
iae_single, y_single = simulate(ctrl.DiscretePID(outer_gains(), TS), False)

cp = ctrl.CascadeParams()
cp.spMin, cp.spMax = -5.0, 5.0
casc = ctrl.CascadeController(ctrl.DiscretePID(outer_gains(), TS),
                              ctrl.DiscretePID(inner_gains(), TS), cp, TS)
iae_casc, y_casc = simulate(casc, True)

print(f"\n-- Inner-loop load step of {D_MAG} at k={K_DIST} --")
print(f"  single-loop PID : IAE = {iae_single:8.4f}   final y = {y_single:.4f}")
print(f"  cascade PID/PID : IAE = {iae_casc:8.4f}   final y = {y_casc:.4f}")
print(f"  improvement     : {100.0 * (1.0 - iae_casc / iae_single):.2f} %")
print(f"  inner setpoint held at {casc.inner_setpoint():.4f} "
      f"(clamped this step: {casc.setpoint_clamped()})")

reject_ok = np.isfinite(iae_casc) and iae_casc < iae_single

# ---------------------------------------------------------------------------
# 2. Setpoint clamp: a tight spMax must bound the inner setpoint
# ---------------------------------------------------------------------------
cp_tight = ctrl.CascadeParams()
cp_tight.spMin, cp_tight.spMax = -0.3, 0.3
casc_tight = ctrl.CascadeController(ctrl.DiscretePID(outer_gains(), TS),
                                    ctrl.DiscretePID(inner_gains(), TS), cp_tight, TS)
sp_peak = 0.0
x1 = x2 = 0.0
for k in range(N):
    casc_tight.set_inner_measurement(x1)
    u = casc_tight.compute(REF - x2)
    sp_peak = max(sp_peak, abs(casc_tight.inner_setpoint()))
    x2 = A2 * x2 + (1.0 - A2) * x1
    x1 = A1 * x1 + (1.0 - A1) * u

print(f"\n-- Setpoint clamp at +/-0.3 --")
print(f"  peak |inner setpoint| = {sp_peak:.6f}   (limit 0.300000)")

clamp_ok = sp_peak <= 0.3 + 1e-9

# ---------------------------------------------------------------------------
# 3. SMC inner loop: the sign convention is flipped automatically
# ---------------------------------------------------------------------------
sp = ctrl.SMCParams()
sp.c_e, sp.c_de, sp.K, sp.phi = 1.0, 5.0 * TS, 6.0, 0.30
sp.uMin, sp.uMax = -10.0, 10.0
casc_smc = ctrl.CascadeController(ctrl.DiscretePID(outer_gains(), TS),
                                  ctrl.DiscreteSMC(sp, TS), cp, TS)
x1 = x2 = 0.0
for k in range(N):
    casc_smc.set_inner_measurement(x1)
    u = casc_smc.compute(REF - x2)
    x2 = A2 * x2 + (1.0 - A2) * x1
    x1 = A1 * x1 + (1.0 - A1) * u

err_smc = abs(REF - x2)
print(f"\n-- PID outer + SMC inner (inner wants e = y - r) --")
print(f"  final |r - y| = {err_smc:.6f}   (no manual sign flip in this script)")

smc_ok = np.isfinite(x2) and err_smc < 0.15

if not (reject_ok and clamp_ok and smc_ok):
    print("\n[FAIL] CascadeController demo did not meet its criteria.")
    sys.exit(1)

print("\n[PASS] CascadeController demo complete.")
