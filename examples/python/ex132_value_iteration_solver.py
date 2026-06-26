"""
ex132_value_iteration_solver.py

Phase 4 (OC2): Grid-based dynamic programming / value iteration - pendulum swing-up.

Mirrors ex115_value_iteration_solver.cpp -- solves the same undamped-pendulum swing-up problem
(start hanging down at theta=0, reach upright theta=pi) via discretized-state-space value
iteration.
"""

import sys
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'ValueIterationSolver'):
        raise AttributeError("ValueIterationSolver not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

Ts, ml2, mgl = 0.01, 1.0, 9.8


def f_dyn(x, u):
    theta_ddot = (u[0] - mgl * np.sin(x[0])) / ml2
    theta_dot_next = x[1] + Ts * theta_ddot
    theta_next = x[0] + Ts * theta_dot_next
    return np.array([theta_next, theta_dot_next])


def cost_fn(x, u):
    return float((1.0 + np.cos(x[0])) + 0.05 * x[1] ** 2 + 0.001 * u[0] ** 2)


gp = ctrl.DPGridParams()
# theta in [0, 2*pi] (not [-pi, pi]) so the upright target theta=pi sits in the grid's
# interior, not at its edge -- see ex115_value_iteration_solver.cpp for the full rationale.
gp.x_min = np.array([0.0, -8.0])
gp.x_max = np.array([2.0 * np.pi, 8.0])
gp.n_grid_per_dim = np.array([41, 41])
gp.u_min = np.array([-15.0])
gp.u_max = np.array([15.0])
gp.n_grid_u = 15
gp.discount = 0.97
gp.max_iter = 400
gp.tol = 1e-4
gp.out_of_grid_penalty = 1e5

vi = ctrl.ValueIterationSolver(f_dyn, cost_fn, gp)
vi.solve()
print(f"solve(): converged={vi.converged()} iterations={vi.iterations()} "
      f"finalDelta={vi.final_delta():.6g}")

state = np.zeros(2)  # hanging straight down
for _ in range(1500):
    u = vi.policy(state)
    state = f_dyn(state, u)

print(f"Final state: theta={state[0]:.4f} theta_dot={state[1]:.4f} (target theta=pi={np.pi:.4f})")

ok = bool(vi.converged() and np.all(np.isfinite(state)) and
          np.cos(state[0]) < -0.9 and abs(state[1]) < 1.5)
print("[PASS]" if ok else "[FAIL]")
sys.exit(0 if ok else 1)
