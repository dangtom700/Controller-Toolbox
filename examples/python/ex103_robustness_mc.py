"""
ex103_robustness_mc.py

Robustness Phase 1: Monte-Carlo closed-loop robustness analysis.

Spawns an ensemble of perturbed first-order plants around a nominal model,
closes each one around a fixed proportional controller, and aggregates
closed-loop robustness statistics. Compares a small-uncertainty ensemble
against a large-uncertainty ensemble.

    Nominal plant:  x[k+1] = 0.6 x[k] + 0.4 u[k],  y = x   (open-loop stable)
    Controller:     u = 0.5 * e   (static gain; closed-loop pole 0.4)
"""

import sys
import os
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    for name in ('MonteCarloResult', 'monte_carlo_analysis', 'spawn_ss_samples'):
        if not hasattr(ctrl, name):
            raise AttributeError(f"{name} not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)


Ts = 0.1
nominal = ctrl.StateSpace(np.array([[0.6]]), np.array([[0.4]]),
                          np.array([[1.0]]), np.array([[0.0]]), Ts)
# Static-gain controller as a 1-state realisation of D = 0.5.
controller = ctrl.StateSpace(np.zeros((1, 1)), np.zeros((1, 1)),
                             np.zeros((1, 1)), np.array([[0.5]]), Ts)


def summarise(tag, r):
    print(f"=== {tag} ===")
    print(f"  samples={r.n_samples}  unstable={r.n_unstable}  "
          f"P(unstable)={r.instability_probability:.4f}")
    print(f"  ||S||_inf  mean={r.sensitivity_peak_stats.mean:.4f}  "
          f"p95={r.sensitivity_peak_stats.p95:.4f}  "
          f"worst={r.sensitivity_peak_stats.worst:.4f}")
    print(f"  GM[dB]     mean={r.gm_stats.mean:.4f}  worst={r.gm_stats.worst:.4f}")
    print(f"  nu-gap     mean={r.nu_gap_stats.mean:.4f}  worst={r.nu_gap_stats.worst:.4f}")


res_small = ctrl.monte_carlo_analysis(nominal, controller, 500, 0.05, seed=42)
summarise("A small sigma (0.05)", res_small)

res_large = ctrl.monte_carlo_analysis(nominal, controller, 500, 0.40, seed=42)
summarise("B large sigma (0.40)", res_large)

# ---- Internal consistency checks (printed, do not abort the example) --------
ok = True
if res_small.n_unstable != 0:
    print("[FAIL] small-sigma ensemble should be fully stable")
    ok = False
if not np.isfinite(res_small.sensitivity_peak_stats.mean):
    print("[FAIL] sensitivity stats not finite")
    ok = False
if res_large.sensitivity_peak_stats.p95 < res_small.sensitivity_peak_stats.p95:
    print("[FAIL] larger uncertainty should not shrink the sensitivity spread")
    ok = False
if res_large.nu_gap_stats.mean < res_small.nu_gap_stats.mean:
    print("[FAIL] larger uncertainty should not shrink the nu-gap")
    ok = False

# spawn_ss_samples returns the ensemble directly for custom evaluation.
ens = ctrl.spawn_ss_samples(nominal, 4, 0.1, seed=1)
if len(ens) != 4:
    print("[FAIL] spawn_ss_samples returned wrong count")
    ok = False

print("[PASS] All checks passed." if ok else "[FAIL] One or more checks failed.")
