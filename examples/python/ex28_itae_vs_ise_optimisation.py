"""
ex28 - ITAE vs ISE Cost Function Comparison
=============================================
Goal     : Optimise PID gains separately using ISE and ITAE cost functions and
           compare the resulting closed-loop transient behaviour.
           ITAE should produce better settling time; ISE often gives faster rise.

Data generation : 1 000-step closed-loop per Nelder-Mead eval; 2 000-step validation.
Verification    :
  - ITAE-optimised: ITAE_val < ISE-optimised ITAE_val (ITAE specialises in settling).
  - ISE-optimised:  ISE_val  < ITAE-optimised ISE_val  (ISE specialises in early error).
  - Both optimised controllers are stable and have SS error < 1%.

Run:
    conda activate soft_robotics
    python ex28_itae_vs_ise_optimisation.py
"""

import numpy as np
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from utils.plant import example_plant, ss_step
from utils.controllers import DiscretePID
from utils.verify import ise, itae, print_summary

Ts       = 0.01
SIM_OPT  = 800
SIM_VAL  = 2000

def run_closed_loop(Kp, Ki, Kd, steps=SIM_OPT):
    pid   = DiscretePID(Kp=Kp, Ki=Ki, Kd=Kd, Ts=Ts, u_min=-10.0, u_max=10.0)
    plant = example_plant()
    y = np.zeros(steps)
    for k in range(steps):
        u = pid.compute(1.0, y[k-1] if k > 0 else 0.0)
        y[k] = ss_step(plant, u)
    return y

def cost_ise(params):
    Kp, Ki, Kd = params
    if Kp <= 0 or Ki < 0 or Kd < 0: return 1e6
    return ise(1.0 - run_closed_loop(Kp, Ki, Kd), Ts)

def cost_itae(params):
    Kp, Ki, Kd = params
    if Kp <= 0 or Ki < 0 or Kd < 0: return 1e6
    return itae(1.0 - run_closed_loop(Kp, Ki, Kd), Ts)

def nelder_mead(f, x0, bounds, max_evals=250, tol=1e-5):
    a, g, r, s = 1.0, 2.0, 0.5, 0.5
    n = len(x0)
    simp = [np.array(x0, dtype=float)]
    for i in range(n):
        v = np.array(x0, dtype=float); v[i] *= 1.05
        v[i] = np.clip(v[i], bounds[i][0], bounds[i][1]); simp.append(v)
    costs = [f(v) for v in simp]
    for _ in range(max_evals - n - 1):
        order = np.argsort(costs); simp = [simp[i] for i in order]; costs = [costs[i] for i in order]
        if np.max([np.max(np.abs(v - simp[0])) for v in simp[1:]]) < tol: break
        c = np.mean(simp[:-1], axis=0)
        xr = np.clip(c + a*(c - simp[-1]), [b[0] for b in bounds], [b[1] for b in bounds])
        fr = f(xr)
        if fr < costs[0]:
            xe = np.clip(c + g*(xr - c), [b[0] for b in bounds], [b[1] for b in bounds])
            fe = f(xe)
            simp[-1], costs[-1] = (xe, fe) if fe < fr else (xr, fr)
        elif fr < costs[-2]:
            simp[-1], costs[-1] = xr, fr
        else:
            xc = np.clip(c + r*(simp[-1] - c), [b[0] for b in bounds], [b[1] for b in bounds])
            fc = f(xc)
            if fc < costs[-1]: simp[-1], costs[-1] = xc, fc
            else:
                for i in range(1, n+1):
                    simp[i] = np.clip(simp[0] + s*(simp[i] - simp[0]),
                                      [b[0] for b in bounds], [b[1] for b in bounds])
                    costs[i] = f(simp[i])
    return simp[0], costs[0]

print("=" * 60)
print("ex28 - ITAE vs ISE Optimisation")
print("=" * 60)

x0     = [2.0, 1.0, 0.3]
bounds = [(0.01, 20.0), (0.0, 20.0), (0.0, 5.0)]

print("\n  Optimising for ISE ...")
x_ise, _ = nelder_mead(cost_ise, x0, bounds, max_evals=250)
print("  Optimising for ITAE ...")
x_itae, _ = nelder_mead(cost_itae, x0, bounds, max_evals=250)

print(f"\n  ISE-opt:  Kp={x_ise[0]:.4f},  Ki={x_ise[1]:.4f},  Kd={x_ise[2]:.4f}")
print(f"  ITAE-opt: Kp={x_itae[0]:.4f}, Ki={x_itae[1]:.4f}, Kd={x_itae[2]:.4f}")

y_ise  = run_closed_loop(*x_ise,  steps=SIM_VAL)
y_itae = run_closed_loop(*x_itae, steps=SIM_VAL)

e_ise  = 1.0 - y_ise
e_itae = 1.0 - y_itae

print(f"\n  Validation metrics (ISE-opt vs ITAE-opt):")
print(f"  {'Metric':<10} | {'ISE-opt':>12} | {'ITAE-opt':>12}")
for name, f_fn in [("ISE", ise), ("ITAE", itae)]:
    v_ise  = f_fn(e_ise,  Ts)
    v_itae = f_fn(e_itae, Ts)
    print(f"  {name:<10} | {v_ise:>12.5f} | {v_itae:>12.5f}")

ise_ise   = ise(e_ise,   Ts)
ise_itae  = ise(e_itae,  Ts)
itae_ise  = itae(e_ise,  Ts)
itae_itae = itae(e_itae, Ts)

results = {}
results["ise_opt_better_ise"]   = ise_ise   < ise_itae
results["itae_opt_better_itae"] = itae_itae < itae_ise
results["ise_stable"]  = np.all(np.isfinite(y_ise))  and float(np.max(np.abs(y_ise)))  < 10.0
results["itae_stable"] = np.all(np.isfinite(y_itae)) and float(np.max(np.abs(y_itae))) < 10.0

ss_e_ise  = abs(float(np.mean(y_ise[-200:])) - 1.0)
ss_e_itae = abs(float(np.mean(y_itae[-200:])) - 1.0)
results["ise_ss"]  = ss_e_ise  < 0.01
results["itae_ss"] = ss_e_itae < 0.01
print(f"\n  ISE-opt SS error: {ss_e_ise:.4f}, ITAE-opt SS error: {ss_e_itae:.4f}")

print_summary(results)
