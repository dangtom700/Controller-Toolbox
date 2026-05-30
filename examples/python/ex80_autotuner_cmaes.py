"""
ex80_autotuner_cmaes.py
------------------------
Part 22: AutoTuner (CMA-ES) binding demonstration.

Uses AutoTuner to find PID gains [Kp, Ki, Kd] that minimise closed-loop IAE
for a 2nd-order plant G(s) = 1/(s+1)^2 discretised at Ts=0.05s.

Acceptance:
  - Tuned IAE < default IAE * 0.90  (at least 10% improvement)
  - All tuned gains within specified bounds

Expected output: PASS
"""
import sys
import numpy as np

sys.path.insert(0, __file__.rsplit("\\", 1)[0])
import _setup_bindings
try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'AutoTuner'):
        raise AttributeError("AutoTuner not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

Ts = 0.05
N_sim = 100
ref = 1.0

# Plant: (s+1)^2 discretised via ZOH
sys_c = ctrl.StateSpace(
    np.array([[0., 1.], [-1., -2.]]),
    np.array([[0.], [1.]]),
    np.array([[1., 0.]]),
    np.array([[0.]]),
    0.0)
plant = ctrl.c2d(sys_c, Ts, ctrl.C2dMethod.ZOH)

def simulate_iae(params):
    Kp, Ki, Kd = float(params[0]), float(params[1]), float(params[2])
    pp = ctrl.PIDParams()
    pp.Kp = Kp; pp.Ki = Ki; pp.Kd = Kd
    pp.N = 10.0; pp.uMin = -10.0; pp.uMax = 10.0
    pid = ctrl.DiscretePID(pp, Ts)
    x = np.zeros(plant.A.shape[0])
    iae = 0.0
    for _ in range(N_sim):
        y = float((plant.C @ x)[0])
        e = ref - y
        u = pid.compute(e)
        x = plant.A @ x + (plant.B * u).flatten()
        iae += abs(e) * Ts
    return iae

# Default baseline
x0 = np.array([1.0, 0.1, 0.0])
iae_default = simulate_iae(x0)
print(f"Default  Kp={x0[0]:.2f} Ki={x0[1]:.2f} Kd={x0[2]:.2f}  IAE={iae_default:.4f}")

# CMA-ES tuning
atp = ctrl.AutoTunerParams()
atp.n = 3
atp.sigma0 = 0.4
atp.maxIter = 100
atp.lower = np.array([0.01, 0.01, 0.0])
atp.upper = np.array([5.0,  2.0,  1.0])

tuner = ctrl.AutoTuner(atp, 42)
result = tuner.tune(simulate_iae, x0)

iae_tuned = result.cost
p = result.params
print(f"Tuned    Kp={p[0]:.3f} Ki={p[1]:.3f} Kd={p[2]:.3f}  IAE={iae_tuned:.4f}")
print(f"n_evals={result.n_evals}  n_gens={result.n_gens}  converged={result.converged}")
print(f"IAE improvement: {(iae_default - iae_tuned) / iae_default * 100:.1f}%")

improved  = iae_tuned < iae_default * 0.90
in_bounds = (p >= atp.lower - 1e-9).all() and (p <= atp.upper + 1e-9).all()

if improved and in_bounds:
    print("PASS")
else:
    if not improved:
        print(f"FAIL: no improvement  tuned={iae_tuned:.4f} >= 0.9*default={iae_default*0.9:.4f}")
    if not in_bounds:
        print(f"FAIL: params out of bounds: {p}")
    sys.exit(1)
