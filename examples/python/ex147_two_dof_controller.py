"""
ex147_two_dof_controller.py -- TwoDOFController: functional feedforward + feedback trim.

Demonstrates:
  1. A Python callable f(r, d) -> u_ff as the feedforward, so the term can be a physics
     inversion or a measured-disturbance signal - not just a designed G_ff(z) filter
     (which is what FeedforwardController requires).
  2. With an exact plant inverse the feedback trim decays to ~0: the feedforward is
     carrying the load and the integrator has nothing left to do.
  3. A MEASURED disturbance is cancelled before the feedback controller sees an error.
  4. A deliberately WRONG feedforward still tracks - the feedback trim absorbs the
     modelling error, which is the whole point of the 2-DOF split.

Plant:  y[k+1] = a*y[k] + (1-a)*K_p*(u[k] + d[k]),  K_p = 2.5, tau = 1 s

Sign convention: compute(r - y)  (same as DiscretePID).
"""

import sys
import _setup_bindings  # noqa: F401

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'TwoDOFController'):
        raise AttributeError("TwoDOFController not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

import numpy as np

print("=== ex147: TwoDOFController (feedforward + feedback) ===")

TS = 0.05
N = 400
K_PLANT = 2.5
REF = 1.0
A = np.exp(-TS / 1.0)


def trim_gains():
    p = ctrl.PIDParams()
    p.Kp, p.Ki, p.Kd = 0.6, 0.30, 0.0
    p.uMin, p.uMax = -5.0, 5.0
    return p


def two_dof(ff):
    tp = ctrl.TwoDOFParams()
    tp.uMin, tp.uMax = -5.0, 5.0
    return ctrl.TwoDOFController(ctrl.DiscretePID(trim_gains(), TS), ff, tp, TS)


def settle_index(ys, ref, tol=0.02):
    """First index after which |ref - y| stays within tol*|ref|."""
    idx = len(ys)
    for k in range(len(ys) - 1, -1, -1):
        if abs(ref - ys[k]) > tol * abs(ref):
            break
        idx = k
    return idx


# ---------------------------------------------------------------------------
# 1. Feedback only vs 2-DOF with the exact inverse u_ff = r / K_p
# ---------------------------------------------------------------------------
fb_only = ctrl.DiscretePID(trim_gains(), TS)
y, ys_fb = 0.0, []
for k in range(N):
    u = fb_only.compute(REF - y)
    y = A * y + (1.0 - A) * K_PLANT * u
    ys_fb.append(y)

c2 = two_dof(lambda r, d: r / K_PLANT - d)
c2.set_reference(REF)
y, ys_2d = 0.0, []
for k in range(N):
    u = c2.compute(REF - y)
    y = A * y + (1.0 - A) * K_PLANT * u
    ys_2d.append(y)

s_fb, s_2d = settle_index(ys_fb, REF), settle_index(ys_2d, REF)
print(f"\n-- Step response (plant DC gain {K_PLANT}) --")
print(f"  feedback only : settles at k = {s_fb:3d}  (t = {s_fb * TS:.2f} s)")
print(f"  2-DOF         : settles at k = {s_2d:3d}  (t = {s_2d * TS:.2f} s)")
print(f"  final u_ff = {c2.feedforward_term():.6f}  (exact inverse r/K = {REF / K_PLANT:.6f})")
print(f"  final u_fb = {c2.feedback_term():.6f}  (-> 0 when the feedforward is exact)")

faster_ok = s_2d < s_fb
handover_ok = abs(c2.feedback_term()) < 0.02

# ---------------------------------------------------------------------------
# 2. Measured-disturbance feedforward
# ---------------------------------------------------------------------------
c2d_ = two_dof(lambda r, d: r / K_PLANT - d)
c2d_.set_reference(REF)
y, peak = 0.0, 0.0
D_LOAD = 0.2
for k in range(N):
    d = D_LOAD if k >= 200 else 0.0
    c2d_.set_measured_disturbance(d)
    u = c2d_.compute(REF - y)
    y = A * y + (1.0 - A) * K_PLANT * (u + d)
    if k >= 200:
        peak = max(peak, abs(REF - y))

print(f"\n-- Measured load step of {D_LOAD} at k=200 --")
print(f"  peak |r - y| after the step = {peak:.6f}")
dff_ok = np.isfinite(y) and peak < 0.02

# ---------------------------------------------------------------------------
# 3. Wrong feedforward: the trim must still bring the loop home
# ---------------------------------------------------------------------------
print("\n-- Feedforward gain error sweep (integrator absorbs the mismatch) --")
print(f"  {'assumed K':>12}{'final y':>12}{'final u_ff':>13}{'final u_fb':>13}")
robust_ok = True
for k_assumed in (1.5, 2.5, 4.0):
    c = two_dof(lambda r, d, ka=k_assumed: r / ka - d)
    c.set_reference(REF)
    y = 0.0
    for _ in range(N * 3):
        u = c.compute(REF - y)
        y = A * y + (1.0 - A) * K_PLANT * u
    print(f"  {k_assumed:>12.2f}{y:>12.5f}{c.feedforward_term():>13.5f}"
          f"{c.feedback_term():>13.5f}")
    robust_ok = robust_ok and np.isfinite(y) and abs(REF - y) < 0.01

if not (faster_ok and handover_ok and dff_ok and robust_ok):
    print("\n[FAIL] TwoDOFController demo did not meet its criteria.")
    sys.exit(1)

print("\n[PASS] TwoDOFController demo complete.")
