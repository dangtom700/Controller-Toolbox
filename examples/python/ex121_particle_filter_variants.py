"""
ex121_particle_filter_variants.py

Phase 3 Roadmap Phase 2 (EF3): Bootstrap vs Auxiliary vs Rao-Blackwellized PF.

Mirrors ex104_particle_filter_variants.cpp - compares all three variants' RMSE on a
linear-Gaussian velocity substate additively coupled to a nonlinear angle measurement.
"""

import sys
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'ParticleFilterV2'):
        raise AttributeError("ParticleFilterV2 not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

Ts = 0.1
N = 150
N_PARTICLES = 40


def f(x, u):
    return np.array([x[0] + x[1] * Ts + u[0], 0.0])


def h(x, _u):
    return np.array([np.sin(x[0]) + x[1]])


A_lin = np.array([[1.0]])
B_lin = np.array([[0.0]])
C_lin = np.array([[1.0]])
Q_lin = np.array([[0.001]])
R_lin = np.array([[0.05]])

rng = np.random.default_rng(11)
theta, v = 0.0, 0.5
v_true = np.zeros(N)
y_meas = np.zeros(N)
for k in range(N):
    v += rng.normal(0.0, 0.01)
    theta += v * Ts
    v_true[k] = v
    y_meas[k] = np.sin(theta) + v + rng.normal(0.0, np.sqrt(0.05))


def run_variant(variant):
    p = ctrl.ParticleFilterParamsV2()
    p.n_particles = N_PARTICLES
    p.Q = np.eye(2) * 0.001
    p.R = np.array([[0.05]])
    p.seed = 21
    p.variant = variant
    p.linear_state_indices = [1]

    pf = ctrl.ParticleFilterV2(p, 2, 1, f, h, A_lin, B_lin, C_lin, Q_lin, R_lin)
    pf.initialise(np.zeros(2))

    v_est = np.zeros(N)
    u0 = np.zeros(1)
    for k in range(N):
        pf.step(np.array([y_meas[k]]), u0)
        v_est[k] = pf.state()[1]
    return float(np.sqrt(np.mean((v_est - v_true) ** 2)))


rmse_bootstrap = run_variant(ctrl.PFVariant.Bootstrap)
rmse_auxiliary = run_variant(ctrl.PFVariant.Auxiliary)
rmse_rb = run_variant(ctrl.PFVariant.RaoBlackwellized)

print(f"Velocity RMSE @ N={N_PARTICLES} particles:")
print(f"  Bootstrap:        {rmse_bootstrap:.4f}")
print(f"  Auxiliary:        {rmse_auxiliary:.4f}")
print(f"  RaoBlackwellized: {rmse_rb:.4f}")

ok = (np.isfinite(rmse_bootstrap) and np.isfinite(rmse_auxiliary) and np.isfinite(rmse_rb)
      and rmse_rb < rmse_bootstrap)
print("[PASS] All checks passed." if ok else "[FAIL] One or more checks failed.")
sys.exit(0 if ok else 1)
