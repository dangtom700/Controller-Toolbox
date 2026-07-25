"""
ex149_fuzzy_smc.py -- FuzzySlidingModeController: fuzzy-scheduled SMC.

Demonstrates:
  1. Mamdani inference on the sliding surface (s, s_dot) retuning the switching gain K
     and boundary layer phi every step.
  2. Matched-authority comparison against a fixed-gain DiscreteSMC: the fixed one runs
     at K=8, the FSMC's nominal K = 8/(1+gainSpan) grows back to 8 at large |s|. Same
     reaching gain, but the FSMC relaxes it inside the boundary layer.
  3. Lower total control variation TV = sum |u[k] - u[k-1]| (less actuator wear) at
     comparable or better tracking.
  4. The scheduled gain never leaves [Kmin, Kmax].

Plant: y[k+1] = a*y[k] + (1-a)*(u[k] + d[k]),  tau = 0.2 s
       with a MATCHED sinusoidal disturbance d[k] = 0.3*sin(2*pi*t).

Sign convention: compute(y - r)  -- REVERSED from DiscretePID, inherited from DiscreteSMC.
"""

import sys
import _setup_bindings  # noqa: F401

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'FuzzySlidingModeController'):
        raise AttributeError("FuzzySlidingModeController not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

import numpy as np

print("=== ex149: FuzzySlidingModeController (fuzzy-scheduled SMC) ===")

TS = 0.01
N = 2000
REF = 1.0
K_REACH = 8.0
GAIN_SPAN = 0.8
A = np.exp(-TS / 0.2)


def base_smc_params():
    p = ctrl.SMCParams()
    p.c_e, p.c_de = 1.0, 5.0 * TS      # c_de = lambda*Ts, lambda = 5 [1/s]
    p.K, p.phi = K_REACH, 0.05
    p.uMin, p.uMax = -20.0, 20.0
    return p


def fsmc_params(gain_span=GAIN_SPAN, phi_span=0.5):
    p = ctrl.FuzzySMCParams()
    p.smc = base_smc_params()
    p.smc.K = K_REACH / (1.0 + gain_span)   # grows back to K_REACH at m = 1
    p.fuzzy.e_scale = 0.5      # normalises the SLIDING SURFACE s, not the error
    p.fuzzy.de_scale = 20.0    # normalises s_dot = (s[k]-s[k-1])/Ts
    p.fuzzy.u_scale = 1.0      # modulation universe: m = |fuzzy| / u_scale
    p.gainSpan, p.phiSpan = gain_span, phi_span
    p.Kmin, p.Kmax = 0.5, 20.0
    p.phiMin, p.phiMax = 0.01, 1.0
    return p


def simulate(controller, track_gain):
    """Returns (IAE over the 2nd half, TV of u, K range, phi range)."""
    y = u_prev = tv = iae = 0.0
    k_lo, k_hi, p_lo, p_hi = 1e9, -1e9, 1e9, -1e9
    for k in range(N):
        u = controller.compute(y - REF)     # SMC convention: e = y - r
        tv += abs(u - u_prev)
        u_prev = u
        if k > N // 2:
            iae += abs(REF - y) * TS
        if track_gain:
            k_lo, k_hi = min(k_lo, controller.switching_gain()), max(k_hi, controller.switching_gain())
            p_lo, p_hi = min(p_lo, controller.boundary_layer()), max(p_hi, controller.boundary_layer())
        y = A * y + (1.0 - A) * (u + 0.3 * np.sin(2.0 * np.pi * k * TS))
    return iae, tv, (k_lo, k_hi), (p_lo, p_hi)


# ---------------------------------------------------------------------------
# 1. Fixed-gain SMC vs fuzzy-scheduled SMC
# ---------------------------------------------------------------------------
iae_fix, tv_fix, _, _ = simulate(ctrl.DiscreteSMC(base_smc_params(), TS), False)
fp = fsmc_params()
fsmc = ctrl.FuzzySlidingModeController(fp, TS)
iae_fz, tv_fz, k_rng, p_rng = simulate(fsmc, True)

print("\n-- Matched reaching authority (both reach at K = 8) --")
print(f"  {'':<22}{'IAE (2nd half)':>16}{'TV of u':>14}")
print(f"  {'DiscreteSMC  K=8':<22}{iae_fix:>16.4f}{tv_fix:>14.2f}")
print(f"  {'FuzzySMC  K=4.4->8':<22}{iae_fz:>16.4f}{tv_fz:>14.2f}")
print(f"\n  control-variation reduction : {100.0 * (1.0 - tv_fz / tv_fix):.2f} %")
print(f"  K   ranged over [{k_rng[0]:.4f}, {k_rng[1]:.4f}]   bounds "
      f"[{fp.Kmin:.2f}, {fp.Kmax:.2f}]")
print(f"  phi ranged over [{p_rng[0]:.4f}, {p_rng[1]:.4f}]   bounds "
      f"[{fp.phiMin:.2f}, {fp.phiMax:.2f}]")

tv_ok = np.isfinite(tv_fz) and tv_fz < tv_fix
track_ok = iae_fz < 2.0 * iae_fix
sched_ok = (k_rng[1] - k_rng[0]) > 1e-3 and k_rng[0] >= fp.Kmin - 1e-9 and k_rng[1] <= fp.Kmax + 1e-9

# ---------------------------------------------------------------------------
# 2. gainSpan sweep: more modulation depth, less chattering
# ---------------------------------------------------------------------------
print("\n-- gainSpan sweep (nominal K rescaled so the reaching gain stays 8) --")
print(f"  {'gainSpan':>10}{'nominal K':>12}{'IAE':>12}{'TV of u':>14}")
tv_by_span = []
for gs in (0.2, 0.5, 0.8, 1.2):
    p = fsmc_params(gain_span=gs)
    iae_s, tv_s, _, _ = simulate(ctrl.FuzzySlidingModeController(p, TS), True)
    tv_by_span.append(tv_s)
    print(f"  {gs:>10.2f}{p.smc.K:>12.4f}{iae_s:>12.4f}{tv_s:>14.2f}")

sweep_ok = all(np.isfinite(v) for v in tv_by_span) and tv_by_span[-1] < tv_by_span[0]

# ---------------------------------------------------------------------------
# 3. Parameter validation
# ---------------------------------------------------------------------------
threw = 0
for mutate in (lambda p: setattr(p, 'phiMin', 0.0),      # boundary layer must be > 0
               lambda p: setattr(p, 'gainSpan', -1.0),   # multiplier would hit zero
               lambda p: setattr(p, 'Kmax', 0.1)):       # Kmax below Kmin
    bad = fsmc_params()
    mutate(bad)
    try:
        ctrl.FuzzySlidingModeController(bad, TS)
    except (ValueError, RuntimeError):
        threw += 1
print(f"\n-- Validation --\n  invalid parameter sets rejected : {threw} / 3")

if not (tv_ok and track_ok and sched_ok and sweep_ok and threw == 3):
    print("\n[FAIL] FuzzySlidingModeController demo did not meet its criteria.")
    sys.exit(1)

print("\n[PASS] FuzzySlidingModeController demo complete.")
