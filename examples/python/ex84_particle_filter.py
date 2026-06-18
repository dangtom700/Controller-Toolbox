"""
ex84_particle_filter.py
-----------------------
Part 25: ParticleFilter (SIR) binding demonstration.

Linear plant: x[k+1] = 0.9*x[k] + w[k],  y[k] = x[k] + v[k]
Q=0.01, R=0.25, N=300 particles, 30 steps.

Acceptance:
  - RMSE < 0.30
  - resample_count >= 0 (filter ran without error)

Expected output: PASS
"""
import sys
import numpy as np

sys.path.insert(0, __file__.rsplit("\\", 1)[0])
import _setup_bindings
try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'ParticleFilter'):
        raise AttributeError("ParticleFilter not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

Q_val = 0.01
R_val = 0.25
Ts    = 0.1

pfp = ctrl.ParticleFilterParams()
pfp.n_particles = 300
pfp.Q    = np.array([[Q_val]])
pfp.R    = np.array([[R_val]])
pfp.Ts   = Ts
pfp.seed = 7

def f_proc(x, u):
    return np.array([0.9 * x[0] + u[0]])

def h_meas(x, u):
    return x.copy()

pf = ctrl.ParticleFilter(pfp, 1, 1, f_proc, h_meas)
pf.initialise(np.zeros(1), np.eye(1) * 0.1)

rng = np.random.default_rng(42)
x_true = 0.0
u_zero = np.zeros(1)
sse = 0.0
N   = 30

for k in range(N):
    x_true = 0.9 * x_true + rng.normal(0.0, np.sqrt(Q_val))
    y_meas = x_true + rng.normal(0.0, np.sqrt(R_val))
    pf.step(np.array([y_meas]), u_zero)
    sse += (pf.state()[0] - x_true) ** 2

rmse = np.sqrt(sse / N)
print(f"PF RMSE = {rmse:.4f}  (resample count = {pf.resample_count})")
print(f"N_eff   = {pf.effective_sample_size():.1f}")

if rmse < 0.30:
    print("PASS")
else:
    print(f"FAIL: RMSE {rmse:.4f} >= 0.30")
    sys.exit(1)
