"""
ex95_gp_regression.py -- Gaussian Process Regression demo.

Demonstrates:
  1. GP regression on noisy sine data (SE kernel).
  2. Posterior mean and uncertainty (variance) at test points.
  3. Fixed-budget behaviour: older points evicted when n_max reached.
  4. Online use: GP updated incrementally as new data arrives.

"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'build', 'bindings'))
if sys.platform == "win32" and hasattr(os, "add_dll_directory"):
    for _p in [r"C:\msys64\mingw64\bin"]:
        if os.path.isdir(_p):
            os.add_dll_directory(_p)

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'GaussianProcess'):
        raise AttributeError("GaussianProcess not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

import numpy as np

print("=== ex95: Gaussian Process Regression ===")

rng = np.random.default_rng(7)

# ---------------------------------------------------------------------------
# 1. Fit GP on noisy sine data
# ---------------------------------------------------------------------------
print("\n-- Part 1: fit on y = sin(2x) + noise --")

gp_p = ctrl.GPParams()
gp_p.length_scale = 1.0
gp_p.signal_var   = 1.0
gp_p.noise_var    = 0.04
gp_p.n_max        = 60

gp = ctrl.GaussianProcess(1, gp_p)

x_train = np.linspace(-3.0, 3.0, 40)
for xv in x_train:
    y_noisy = np.sin(2.0 * xv) + 0.2 * float(rng.standard_normal())
    gp.add_point(np.array([xv]), y_noisy)

gp.fit()
print(f"  Training points: {gp.size()}")

# Test points
print(f"\n  {'x':>6}  {'true y':>8}  {'GP mean':>8}  {'GP std':>8}")
errors = []
for xv in np.linspace(-2.5, 2.5, 8):
    pred   = gp.predict(np.array([xv]))
    true_y = np.sin(2.0 * xv)
    errors.append(abs(pred.mean - true_y))
    print(f"  {xv:6.2f}  {true_y:8.4f}  {pred.mean:8.4f}  {np.sqrt(pred.variance):8.4f}")

print(f"  Mean abs error: {np.mean(errors):.4f}  (< 0.30 expected)")

# ---------------------------------------------------------------------------
# 2. Uncertainty is higher away from training data
# ---------------------------------------------------------------------------
print("\n-- Part 2: variance grows far from training data --")
pred_near = gp.predict(np.array([0.0]))   # inside training range
pred_far  = gp.predict(np.array([10.0]))  # far outside
print(f"  Variance at x=0:   {pred_near.variance:.4f}")
print(f"  Variance at x=10:  {pred_far.variance:.4f}  (should be larger)")
assert pred_far.variance > pred_near.variance, "GP uncertainty sanity check failed"

# ---------------------------------------------------------------------------
# 3. Fixed-budget eviction
# ---------------------------------------------------------------------------
print("\n-- Part 3: fixed-budget eviction (n_max=10) --")
gp_small = ctrl.GaussianProcess(1, ctrl.GPParams())
gp_small_p = ctrl.GPParams()
gp_small_p.n_max = 10
gp_small = ctrl.GaussianProcess(1, gp_small_p)

for k in range(25):
    gp_small.add_point(np.array([float(k)]), float(k))

print(f"  Points added: 25,  stored: {gp_small.size()}  (n_max=10, oldest evicted)")
assert gp_small.size() == 10

# ---------------------------------------------------------------------------
# 4. Online incremental update
# ---------------------------------------------------------------------------
print("\n-- Part 4: online incremental GP on plant output --")
gp_online_p = ctrl.GPParams()
gp_online_p.length_scale = 0.5
gp_online_p.signal_var   = 1.0
gp_online_p.noise_var    = 0.01
gp_online_p.n_max        = 30
gp_online = ctrl.GaussianProcess(1, gp_online_p)

x_plant = 0.0
for k in range(50):
    u       = np.sin(k * 0.3)
    y_plant = 0.85 * x_plant + 0.15 * float(u) + 0.05 * float(rng.standard_normal())
    gp_online.add_point(np.array([float(u)]), y_plant)
    x_plant = y_plant
    if (k + 1) % 10 == 0:
        gp_online.fit()
        pred = gp_online.predict(np.array([0.0]))
        print(f"  k={k+1:2d}: size={gp_online.size()}, "
              f"pred(u=0)={pred.mean:.4f} +/- {np.sqrt(pred.variance):.4f}")

print("\n[PASS] GaussianProcess demo complete.")
