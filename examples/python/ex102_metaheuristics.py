"""
ex102_metaheuristics.py

Demonstrates ctrl.GeneticAlgorithm, ctrl.ParticleSwarmOptimizer, and
ctrl.DifferentialEvolution on a 2-D Rosenbrock function.

Global minimum at (1, 1), f=0.
"""

import sys
import os
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    for cls in ('GeneticAlgorithm', 'ParticleSwarmOptimizer', 'DifferentialEvolution'):
        if not hasattr(ctrl, cls):
            raise AttributeError(f"{cls} not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)


def rosenbrock(x):
    """2-D Rosenbrock; min = 0 at (1, 1)."""
    a = 1.0 - float(x[0])
    b = float(x[1]) - float(x[0]) ** 2
    return a * a + 100.0 * b * b


lb = np.array([-2.0, -2.0])
ub = np.array([ 3.0,  3.0])


# ---- GeneticAlgorithm -------------------------------------------------------
gp = ctrl.GAParams()
gp.n_dim = 2;  gp.population = 60;  gp.max_gen = 200
gp.crossover = 0.8;  gp.mutation = 0.05;  gp.alpha = 0.3
gp.lower = lb;  gp.upper = ub;  gp.seed = 42

ga = ctrl.GeneticAlgorithm(gp)
r_ga = ga.optimize(rosenbrock)
print(f"=== GeneticAlgorithm ===")
print(f"  Best params:  [{r_ga.params[0]:.5f}, {r_ga.params[1]:.5f}]")
print(f"  Best cost:    {r_ga.cost:.6f}")
print(f"  Evaluations:  {r_ga.n_evals}")
print(f"  Generations:  {r_ga.n_gens}")
print(f"  Converged:    {r_ga.converged}\n")
assert np.isfinite(r_ga.cost), "GA cost not finite"
assert r_ga.cost < 5.0, f"GA Rosenbrock cost unexpectedly high: {r_ga.cost}"

# ---- ParticleSwarmOptimizer -------------------------------------------------
pp = ctrl.PSOParams()
pp.n_dim = 2;  pp.n_particles = 40;  pp.max_iter = 200
pp.w = 0.729;  pp.c1 = 1.495;  pp.c2 = 1.495
pp.lower = lb;  pp.upper = ub;  pp.seed = 17

pso = ctrl.ParticleSwarmOptimizer(pp)
r_pso = pso.optimize(rosenbrock)
print(f"=== ParticleSwarmOptimizer ===")
print(f"  Best params:  [{r_pso.params[0]:.5f}, {r_pso.params[1]:.5f}]")
print(f"  Best cost:    {r_pso.cost:.6f}")
print(f"  Evaluations:  {r_pso.n_evals}")
print(f"  Iterations:   {r_pso.n_gens}")
print(f"  Converged:    {r_pso.converged}\n")
assert np.isfinite(r_pso.cost), "PSO cost not finite"
assert r_pso.cost < 5.0, f"PSO Rosenbrock cost unexpectedly high: {r_pso.cost}"

# ---- DifferentialEvolution --------------------------------------------------
dp = ctrl.DEParams()
dp.n_dim = 2;  dp.population = 30;  dp.max_gen = 200
dp.F = 0.8;  dp.CR = 0.9
dp.lower = lb;  dp.upper = ub;  dp.seed = 7

de = ctrl.DifferentialEvolution(dp)
r_de = de.optimize(rosenbrock)
print(f"=== DifferentialEvolution ===")
print(f"  Best params:  [{r_de.params[0]:.5f}, {r_de.params[1]:.5f}]")
print(f"  Best cost:    {r_de.cost:.6f}")
print(f"  Evaluations:  {r_de.n_evals}")
print(f"  Generations:  {r_de.n_gens}")
print(f"  Converged:    {r_de.converged}\n")
assert np.isfinite(r_de.cost), "DE cost not finite"
assert r_de.cost < 5.0, f"DE Rosenbrock cost unexpectedly high: {r_de.cost}"

print("ex102_metaheuristics: all assertions passed.")
