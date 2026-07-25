"""
ex146_disturbance_observer.py -- DisturbanceObserverController: Q-filter DOB + PI.

Demonstrates:
  1. d_hat = Q(z)*(y - y_nom)/gainDC lumps external disturbance AND model error into
     one estimate that is subtracted from the inner controller's command.
  2. Post-step transient beats a bare PI on a plant with gain and pole mismatch.
  3. The Q-filter cutoff omega_q trades rejection speed against noise injection.

Nominal model : G_nom(s) = 1 / (s + 1)      (what the designer has)
True plant    : G(s)     = 1.5 / (s + 0.8)  (gain AND pole mismatch)
Plus an output disturbance step of +0.5 at k = 200.

Sign convention: compute(r - y)  (same as DiscretePID).
"""

import sys
import _setup_bindings  # noqa: F401

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'DisturbanceObserverController'):
        raise AttributeError("DisturbanceObserverController not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

import numpy as np

print("=== ex146: DisturbanceObserverController (Q-filter DOB) ===")

TS = 0.05
N = 900
K_DIST = 200
D_OUT = 0.5
REF = 1.0

A_TRUE = np.exp(-0.8 * TS)
B_TRUE = 1.5 / 0.8 * (1.0 - A_TRUE)

# Nominal model G_nom(s) = 1/(s+1), ZOH-discretised.
NOM = ctrl.c2d(ctrl.StateSpace(np.array([[-1.0]]), np.array([[1.0]]),
                               np.array([[1.0]]), np.array([[0.0]]), 0.0),
               TS, ctrl.C2dMethod.ZOH)


def pi_params():
    p = ctrl.PIDParams()
    p.Kp, p.Ki, p.Kd = 1.5, 0.8, 0.0
    p.uMin, p.uMax = -10.0, 10.0
    return p


def dob_params(omega_q):
    p = ctrl.DOBParams()
    p.omega_q, p.qOrder, p.gainDC = omega_q, 1, 1.0
    p.uMin, p.uMax = -10.0, 10.0
    return p


def simulate(controller, is_dob):
    """Returns (IAE over the 100 samples after the step, final y, final d_hat)."""
    y = iae = 0.0
    for k in range(N):
        d = D_OUT if k >= K_DIST else 0.0
        if is_dob:
            controller.set_plant_output(y)
        u = controller.compute(REF - y)
        if K_DIST <= k < K_DIST + 100:
            iae += abs(REF - y) * TS
        y = A_TRUE * y + B_TRUE * u + (1.0 - A_TRUE) * d
    d_hat = controller.disturbance_estimate() if is_dob else 0.0
    return iae, y, d_hat


# ---------------------------------------------------------------------------
# 1. Bare PI vs PI + DOB
# ---------------------------------------------------------------------------
iae_pi, y_pi, _ = simulate(ctrl.DiscretePID(pi_params(), TS), False)
dob = ctrl.DisturbanceObserverController(ctrl.DiscretePID(pi_params(), TS),
                                         NOM, dob_params(5.0), TS)
iae_dob, y_dob, d_hat = simulate(dob, True)

print(f"\n-- Output disturbance step of {D_OUT} at k={K_DIST}, plant mismatched --")
print(f"  PI alone : IAE(100 post-step) = {iae_pi:.5f}   final y = {y_pi:.5f}")
print(f"  PI + DOB : IAE(100 post-step) = {iae_dob:.5f}   final y = {y_dob:.5f}")
print(f"  improvement : {100.0 * (1.0 - iae_dob / iae_pi):.2f} %")
print(f"  final d_hat : {d_hat:.5f}  (input units; non-zero => observer is loaded)")

transient_ok = np.isfinite(iae_dob) and iae_dob < iae_pi
track_ok = np.isfinite(y_dob) and abs(REF - y_dob) < 0.02
dhat_ok = abs(d_hat) > 1e-3

# ---------------------------------------------------------------------------
# 2. Q-filter bandwidth sweep: faster Q => faster rejection
# ---------------------------------------------------------------------------
print("\n-- Q-filter cutoff sweep --")
print(f"  {'omega_q [rad/s]':>16}{'IAE post-step':>16}{'final d_hat':>14}")
iae_by_wq = []
for wq in (1.0, 3.0, 5.0, 10.0):
    c = ctrl.DisturbanceObserverController(ctrl.DiscretePID(pi_params(), TS),
                                           NOM, dob_params(wq), TS)
    iae_w, _, d_w = simulate(c, True)
    iae_by_wq.append(iae_w)
    print(f"  {wq:>16.1f}{iae_w:>16.5f}{d_w:>14.5f}")

# A faster Q-filter must not be worse than the slowest one.
sweep_ok = all(np.isfinite(v) for v in iae_by_wq) and iae_by_wq[-1] < iae_by_wq[0]

if not (transient_ok and track_ok and dhat_ok and sweep_ok):
    print("\n[FAIL] DisturbanceObserverController demo did not meet its criteria.")
    sys.exit(1)

print("\n[PASS] DisturbanceObserverController demo complete.")
