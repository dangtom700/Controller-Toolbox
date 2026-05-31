"""
ex83_tube_mpc.py
----------------
Part 25: TubeMPC binding demonstration.

Verifies that the tube containment property holds under bounded disturbances:
  max|x[k] - x_nom[k]| <= tubeRadius()  for all k.

Plant: x[k+1] = 0.8*x[k] + 0.2*u[k] + w[k],  y = x
wMax = 0.1, K = -0.3 (tube feedback)

Acceptance:
  - tube containment holds over 50 steps
  - final y within 0.25 of reference r=1

Expected output: PASS
"""
import sys
import numpy as np

sys.path.insert(0, __file__.rsplit("\\", 1)[0])
import _setup_bindings
try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'TubeMPC'):
        raise AttributeError("TubeMPC not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

Ts   = 0.1
wMax = 0.1

# Plant
A = np.array([[0.8]]); B = np.array([[0.2]]); C = np.eye(1); D = np.zeros((1,1))
sys_d = ctrl.StateSpace(A, B, C, D, Ts)

# TubeMPC params
tp = ctrl.TubeMPCParams()
tp.Np = 10; tp.Nu = 3
tp.Q  = 2.0 * np.eye(1)
tp.R  = 0.3 * np.eye(1)
tp.K  = np.array([[-0.3]])
tp.wMax = np.array([wMax])
tp.uMin = np.array([-2.0]); tp.uMax = np.array([2.0])
tp.Ts = Ts

tmpc = ctrl.TubeMPC(sys_d, tp)
z_max = float(tmpc.tube_radius()[0])

rng = np.random.default_rng(42)
x = np.zeros(1)
yref = np.ones(1)

max_err = 0.0
for k in range(50):
    u = tmpc.compute_ref(x, yref)
    x_nom = np.array(tmpc.nominal_state())
    err = abs(x[0] - x_nom[0])
    max_err = max(max_err, err)
    w = rng.uniform(-wMax, wMax)
    x = 0.8 * x + 0.2 * u + np.array([w])

print(f"Tube radius z_max = {z_max:.4f}")
print(f"Max |x - x_nom|   = {max_err:.4f}")
print(f"Final x           = {x[0]:.4f}  (ref = 1.0)")

pass_tube  = max_err <= z_max + 1e-6
pass_track = abs(x[0] - 1.0) < 0.25

if pass_tube and pass_track:
    print("PASS")
else:
    if not pass_tube:
        print(f"FAIL: tube violated (max_err={max_err:.4f} > z_max={z_max:.4f})")
    if not pass_track:
        print(f"FAIL: tracking error {abs(x[0]-1.0):.4f} > 0.25")
    sys.exit(1)
