"""
ex89_deepc.py -- Data-Enabled Predictive Control (DeePC) demo.

Demonstrates:
  1. Generating persistently-exciting offline I/O data (PRBS) from a first-order plant.
  2. Constructing a DeePC controller directly from the raw data (no model ID step).
  3. Closed-loop step tracking and comparison to a PI baseline.

Plant:  y[k+1] = 0.8*y[k] + 0.2*u[k]   DC gain = 1.0,  tau ~ 4.5 steps
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'build', 'bindings'))
if sys.platform == "win32" and hasattr(os, "add_dll_directory"):
    for _p in [r"C:\msys64\mingw64\bin"]:
        if os.path.isdir(_p):
            os.add_dll_directory(_p)

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'DeePC'):
        raise AttributeError("DeePC not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

import numpy as np

# ---------------------------------------------------------------------------
# Plant simulation helper
# ---------------------------------------------------------------------------
def plant_step(state, u, a=0.8, b=0.2):
    y = state[0]
    state[0] = a * state[0] + b * u
    return y


# ---------------------------------------------------------------------------
# 1. Collect offline PRBS data  (N=400 samples, alternating +-1)
# ---------------------------------------------------------------------------
N_OFF   = 400
TS      = 0.1
x_state = [0.0]
u_off   = np.array([1.0 if k % 2 == 0 else -1.0 for k in range(N_OFF)])
y_off   = np.array([plant_step(x_state, u_off[k]) for k in range(N_OFF)])

print(f"Offline data: N={N_OFF}, u in {{{u_off.min():.0f},{u_off.max():.0f}}}, "
      f"y range [{y_off.min():.3f}, {y_off.max():.3f}]")


# ---------------------------------------------------------------------------
# 2. Build DeePC controller
# ---------------------------------------------------------------------------
p = ctrl.DeePCParams()
p.T_ini     = 5
p.Np        = 20
p.rho_y     = 1.0
p.rho_u     = 0.05
p.lambda_g  = 0.5
p.lambda_eq = 1e5
p.uMin      = -3.0
p.uMax      =  3.0
p.rho_admm  = 1.0
p.admm_iter = 150

deepc = ctrl.DeePC(u_off, y_off, p, TS)
print(f"DeePC built: n_col = {N_OFF - p.T_ini - p.Np + 1} Hankel columns")


# ---------------------------------------------------------------------------
# 3. PI baseline
# ---------------------------------------------------------------------------
pi_p = ctrl.PIDParams()
pi_p.Kp = 2.0; pi_p.Ki = 0.5; pi_p.Kd = 0.0
pi_p.N = 10.0; pi_p.Kb = 1.0
pi_p.uMin = -3.0; pi_p.uMax = 3.0
pi = ctrl.DiscretePID(pi_p, TS)


# ---------------------------------------------------------------------------
# 4. Closed-loop simulation
# ---------------------------------------------------------------------------
N_SIM = 200
R     = 1.0

x_dc = [0.0];  x_pi = [0.0]
ys_dc = [];    ys_pi = []
us_dc = [];    us_pi = []

for k in range(N_SIM):
    y_dc = x_dc[0]
    u_dc = deepc.compute_io(y_dc, R)
    plant_step(x_dc, u_dc)
    ys_dc.append(y_dc); us_dc.append(u_dc)

    y_pi = x_pi[0]
    u_pi = float(pi.compute(R - y_pi))
    plant_step(x_pi, u_pi)
    ys_pi.append(y_pi); us_pi.append(u_pi)

ys_dc = np.array(ys_dc);  ys_pi = np.array(ys_pi)
us_dc = np.array(us_dc);  us_pi = np.array(us_pi)


# ---------------------------------------------------------------------------
# 5. Metrics
# ---------------------------------------------------------------------------
t_warmup = p.T_ini + 1
iae_dc   = float(np.sum(np.abs(R - ys_dc[t_warmup:])) * TS)
iae_pi   = float(np.sum(np.abs(R - ys_pi[t_warmup:])) * TS)

print(f"\n{'Step':>5}  {'DeePC y':>9}  {'DeePC u':>8}  |  {'PI y':>8}  {'PI u':>8}")
print("-" * 54)
for k in range(0, min(40, N_SIM)):
    print(f"{k:5d}  {ys_dc[k]:9.4f}  {us_dc[k]:8.4f}  |  "
          f"{ys_pi[k]:8.4f}  {us_pi[k]:8.4f}")

print(f"\nIAE (excl. warm-up):  DeePC = {iae_dc:.4f},  PI = {iae_pi:.4f}")
print(f"Final y:              DeePC = {x_dc[0]:.4f},  PI = {x_pi[0]:.4f}")
print(f"DeePC primal residual (last step): {deepc.last_primal_res():.2e}")
print(f"DeePC is_warmed_up: {deepc.is_warmed_up()}")

# ---------------------------------------------------------------------------
# Internal verification checks
# ---------------------------------------------------------------------------
assert deepc.is_warmed_up(), "[FAIL] DeePC not warmed up after N_SIM steps"
assert np.all(np.isfinite(ys_dc)),  "[FAIL] DeePC produced non-finite outputs"
assert np.all(us_dc >= p.uMin - 1e-9), "[FAIL] DeePC violated uMin"
assert np.all(us_dc <= p.uMax + 1e-9), "[FAIL] DeePC violated uMax"
assert abs(x_dc[0] - R) < 0.15, f"[FAIL] DeePC final value {x_dc[0]:.4f} too far from r={R}"
assert iae_dc < 12.0, f"[FAIL] DeePC IAE={iae_dc:.3f} unexpectedly high"
print("\n[PASS] All DeePC ex89 checks passed.")
