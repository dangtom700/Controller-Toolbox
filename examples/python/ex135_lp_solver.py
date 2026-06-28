"""
ex135_lp_solver.py

LPSolver (Phase 4 OC4) - two-phase simplex on a textbook 2-variable LP.

Mirrors ex118_lp_solver.cpp:
    maximize x1 + x2  (== minimize -x1 - x2)
    s.t.     x1 + 2*x2 <= 4
             3*x1 + x2 <= 6
             x1, x2 >= 0

Known vertex optimum: (x1, x2) = (1.6, 1.2), objective = 2.8 (i.e. c'x = -2.8).
"""

import sys
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'LPSolver'):
        raise AttributeError("LPSolver not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

problem = ctrl.LPProblem()
problem.c = np.array([-1.0, -1.0])
problem.A_ineq = np.array([[1.0, 2.0],
                            [3.0, 1.0]])
problem.b_ineq = np.array([4.0, 6.0])
problem.lb = np.zeros(2)
problem.ub = np.full(2, 1e9)

result = ctrl.LPSolver.solve(problem)

print(f"status={result.status}  x={result.x}  cost={result.cost:.4f}  iters={result.iters}")

ok = (result.status == ctrl.LPStatus.Optimal and
      abs(float(result.x[0]) - 1.6) < 1e-4 and
      abs(float(result.x[1]) - 1.2) < 1e-4 and
      abs(result.cost - (-2.8)) < 1e-4)
print("[PASS]" if ok else "[FAIL]")
sys.exit(0 if ok else 1)
