"""
ex27 - Nelder-Mead Optimisation of PID via ISE
================================================
Goal     : Implement the same Nelder-Mead simplex (alpha=1, γ=2, ρ=0.5, sigma=0.5)
           used in lib/TunerSuite.cpp and optimise PID gains to minimise ISE
           on the closed-loop step response. Verify the optimised ISE is lower
           than the initial-guess ISE.

Data generation : ISE computed from 1 000-step closed-loop per function eval.
Verification    :
  - Optimised ISE < initial ISE.
  - Converged gains within feasible bounds.
  - Simplex terminates within max_evals=300 evaluations.

Run:
    conda activate soft_robotics
    python ex27_nelder_mead_ise.py
"""

import numpy as np
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from utils.plant import example_plant, ss_step
from utils.controllers import DiscretePID
from utils.verify import ise, print_summary

Ts       = 0.01
SIM_STEPS = 1000   # short sim per eval for speed

print("=" * 60)
print("ex27 - Nelder-Mead PID Optimisation (ISE)")
print("=" * 60)

def cost_ise(params):
    Kp, Ki, Kd = params
    if Kp <= 0 or Ki < 0 or Kd < 0:
        return 1e6
    pid   = DiscretePID(Kp=Kp, Ki=Ki, Kd=Kd, Ts=Ts, u_min=-10.0, u_max=10.0)
    plant = example_plant()
    y = np.zeros(SIM_STEPS)
    for k in range(SIM_STEPS):
        u = pid.compute(1.0, y[k-1] if k > 0 else 0.0)
        y[k] = ss_step(plant, u)
    return ise(1.0 - y, Ts)

# Nelder-Mead (matches C++ TunerSuite.cpp parameters)
def nelder_mead(f, x0, bounds, max_evals=300, tol=1e-5):
    alpha, gamma, rho, sigma = 1.0, 2.0, 0.5, 0.5
    n = len(x0)
    simplex = [np.array(x0, dtype=float)]
    for i in range(n):
        v = np.array(x0, dtype=float)
        v[i] *= 1.05
        v[i]  = np.clip(v[i], bounds[i][0], bounds[i][1])
        simplex.append(v)
    costs = [f(s) for s in simplex]
    n_eval = n + 1

    for _ in range(max_evals - n - 1):
        order = np.argsort(costs)
        simplex = [simplex[i] for i in order]
        costs   = [costs[i]   for i in order]

        if np.max([np.max(np.abs(s - simplex[0])) for s in simplex[1:]]) < tol:
            break

        centroid = np.mean(simplex[:-1], axis=0)
        xr = np.clip(centroid + alpha * (centroid - simplex[-1]),
                     [b[0] for b in bounds], [b[1] for b in bounds])
        fr = f(xr); n_eval += 1

        if fr < costs[0]:
            xe = np.clip(centroid + gamma * (xr - centroid),
                         [b[0] for b in bounds], [b[1] for b in bounds])
            fe = f(xe); n_eval += 1
            if fe < fr:
                simplex[-1], costs[-1] = xe, fe
            else:
                simplex[-1], costs[-1] = xr, fr
        elif fr < costs[-2]:
            simplex[-1], costs[-1] = xr, fr
        else:
            xc = np.clip(centroid + rho * (simplex[-1] - centroid),
                         [b[0] for b in bounds], [b[1] for b in bounds])
            fc = f(xc); n_eval += 1
            if fc < costs[-1]:
                simplex[-1], costs[-1] = xc, fc
            else:
                for i in range(1, n + 1):
                    simplex[i] = np.clip(simplex[0] + sigma * (simplex[i] - simplex[0]),
                                         [b[0] for b in bounds], [b[1] for b in bounds])
                    costs[i] = f(simplex[i]); n_eval += 1

    return simplex[0], costs[0], n_eval

x0     = [1.0, 0.5, 0.2]
bounds = [(0.01, 20.0), (0.0, 20.0), (0.0, 5.0)]

ise_initial = cost_ise(x0)
print(f"\n  Initial guess: Kp={x0[0]}, Ki={x0[1]}, Kd={x0[2]}")
print(f"  Initial ISE: {ise_initial:.6f}")

x_opt, ise_opt, n_eval = nelder_mead(cost_ise, x0, bounds, max_evals=300)
Kp_opt, Ki_opt, Kd_opt = x_opt

print(f"\n  Optimised: Kp={Kp_opt:.4f}, Ki={Ki_opt:.4f}, Kd={Kd_opt:.4f}")
print(f"  Optimised ISE: {ise_opt:.6f}  (evals used: {n_eval})")

results = {}
results["ise_improved"]    = ise_opt < ise_initial
results["evals_within_300"] = n_eval <= 300
results["Kp_in_bounds"]   = bounds[0][0] <= Kp_opt <= bounds[0][1]
results["Ki_in_bounds"]   = bounds[1][0] <= Ki_opt <= bounds[1][1]
results["Kd_in_bounds"]   = bounds[2][0] <= Kd_opt <= bounds[2][1]

print(f"\n  {'[PASS]' if results['ise_improved'] else '[FAIL]'} "
      f"ISE improved ({ise_initial:.6f} -> {ise_opt:.6f})")
print(f"  {'[PASS]' if results['evals_within_300'] else '[FAIL]'} "
      f"converged in {n_eval} <= 300 evals")

print_summary(results)
