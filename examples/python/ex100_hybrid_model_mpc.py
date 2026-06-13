"""
ex100_hybrid_model_mpc.py -- H1 / H2 / H4: HybridModel, HybridMPC, HybridModelTrainer.

Demonstrates Phase-2 Hybrid Model algorithms on a spring-mass-damper plant
with unmodeled Coulomb friction.

Plant (true):
    x1_dot =  x2
    x2_dot = -k/m * x1  -  c/m * x2  +  1/m * u  -  Fc/m * sign(x2)
    True params: m=1, k=4, c=0.8, Fc=0.3

Nominal physical model (no friction, k/m=4, c/m=0.8, 1/m=1).

Workflow:
  1. Build HybridModel with physical ODE.
  2. Use HybridModelTrainer to fit a GP data correction from offline data.
  3. Run HybridMPC with online ridge-regression data update and compare
     steady-state IAE against plain NonlinearMPC.
"""

import sys
import os
import numpy as np

# -- ctrl_toolbox import guard -------------------------------------------------
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'build', 'bindings'))
if sys.platform == 'win32' and hasattr(os, 'add_dll_directory'):
    for _p in [r'C:\msys64\mingw64\bin']:
        if os.path.isdir(_p):
            os.add_dll_directory(_p)

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'HybridModel'):
        raise AttributeError("HybridModel not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

# =============================================================================
# True plant: SMD + Coulomb friction
# =============================================================================
def true_xdot(x, u):
    km, cm, inv_m, Fc = 4.0, 0.8, 1.0, 0.3
    sign_v = 1.0 if x[1] > 1e-3 else (-1.0 if x[1] < -1e-3 else 0.0)
    return np.array([x[1],
                     -km*x[0] - cm*x[1] + inv_m*u[0] - Fc*sign_v])

def rk4_true(x, u, Ts):
    k1 = true_xdot(x, u)
    k2 = true_xdot(x + 0.5*Ts*k1, u)
    k3 = true_xdot(x + 0.5*Ts*k2, u)
    k4 = true_xdot(x + Ts*k3, u)
    return x + (Ts/6.0)*(k1 + 2*k2 + 2*k3 + k4)

# =============================================================================
# Nominal physical ODE (no friction)
# =============================================================================
def f_phys(x, u, p):
    return np.array([x[1], -p[0]*x[0] - p[1]*x[1] + p[2]*u[0]])

Ts      = 0.02
p_phys  = np.array([4.0, 0.8, 1.0])   # [k/m, c/m, 1/m]

# =============================================================================
# H1 - Build HybridModel
# =============================================================================
print("\n=== H1  HybridModel ===")

hmp = ctrl.HybridModelParams()
hmp.n_states = 2; hmp.n_inputs = 1; hmp.n_outputs = 2
hmp.Ts = Ts; hmp.rk4_steps = 4

model = ctrl.HybridModel(f_phys, hmp, p_phys=p_phys)
print(f"  state_size={model.state_size()}  input_size={model.input_size()}")
print(f"  has_data_model (before set): {model.has_data_model()}")

x0 = np.zeros(2)
u0 = np.array([1.0])
xn_phys = model.predict_phys(x0, u0)
xn_comb = model.predict(x0, u0)          # same as phys (no data model yet)
assert np.allclose(xn_phys, xn_comb),    "predict should equal predict_phys without data model"

# Attach trivial zero correction and verify transparency
model.set_data_model(lambda x, u: np.zeros(len(x)))
xn_zero = model.predict(x0, u0)
assert np.allclose(xn_phys, xn_zero),    "zero data model should not change prediction"
model.clear_data_model()
print("  H1 construction and predict OK")

# =============================================================================
# H4 - Off-line batch training
# =============================================================================
print("\n=== H4  HybridModelTrainer ===")

# Collect offline training data (open-loop excitation on true plant)
N_train = 120
xs = np.zeros(2)
X_obs  = np.zeros((2, N_train))
U_obs  = np.zeros((1, N_train))
Xn_obs = np.zeros((2, N_train))

rng = np.random.default_rng(42)
for k in range(N_train):
    uk = np.array([1.0 if k < N_train // 2 else -0.5])
    X_obs[:, k]  = xs
    U_obs[:, k]  = uk
    Xn_obs[:, k] = rk4_true(xs, uk, Ts)
    xs           = Xn_obs[:, k]

# Fresh model for training
model_h4 = ctrl.HybridModel(f_phys, hmp, p_phys=p_phys)

# Validate physical-only RMSE
tp_ridge = ctrl.HybridTrainerParams()
tp_ridge.method = ctrl.HybridTrainerMethod.Ridge
trainer_ridge = ctrl.HybridModelTrainer(tp_ridge)

rmse_before = trainer_ridge.validate(model_h4, X_obs, U_obs, Xn_obs)
print(f"  RMSE (physical only): {rmse_before:.6f}")

# Ridge training
res = trainer_ridge.train_hybrid_model(model_h4, X_obs, U_obs, Xn_obs)
rmse_ridge = trainer_ridge.validate(model_h4, X_obs, U_obs, Xn_obs)
print(f"  RMSE after Ridge:     {rmse_ridge:.6f}  (method={res.method}, N={res.n_samples})")
assert res.success,                 "Ridge training failed"
assert np.isfinite(res.train_rmse), "train_rmse not finite"

# GP training
tp_gp = ctrl.HybridTrainerParams()
tp_gp.method           = ctrl.HybridTrainerMethod.GP
tp_gp.gp.length_scale  = 0.5
tp_gp.gp.signal_var    = 0.1
tp_gp.gp.noise_var     = 1e-3
tp_gp.gp.n_max         = N_train
trainer_gp = ctrl.HybridModelTrainer(tp_gp)

res_gp = trainer_gp.train_hybrid_model(model_h4, X_obs, U_obs, Xn_obs)
rmse_gp = trainer_gp.validate(model_h4, X_obs, U_obs, Xn_obs)
print(f"  RMSE after GP:        {rmse_gp:.6f}  (method={res_gp.method})")

print(f"  H4 PASS: Ridge improvement = {rmse_before - rmse_ridge:.6f}")

# =============================================================================
# H2 - HybridMPC closed-loop vs plain NonlinearMPC
# =============================================================================
print("\n=== H2  HybridMPC closed-loop ===")

def make_nmpc_params(Np=10, Nu=3):
    np_ = ctrl.NMPCParams()
    np_.n_states = 2; np_.n_inputs = 1; np_.n_outputs = 2
    np_.Np = Np; np_.Nu = Nu
    np_.rho_y = 10.0; np_.rho_u = 0.1
    np_.uMin = -5.0; np_.uMax = 5.0
    np_.Ts = Ts
    return np_

y_ref = np.array([1.0, 0.0])   # position=1, velocity=0

# ---- HybridMPC (online ridge learning every 20 steps) ----------------------
model_hmpc = ctrl.HybridModel(f_phys, hmp, p_phys=p_phys)
hpars = ctrl.HybridMPCParams()
hpars.nmpc                 = make_nmpc_params()
hpars.data_update_interval = 20
hpars.min_observations     = 10
hpars.ridge_lambda         = 1e-4

hmpc = ctrl.HybridMPC(hpars, model_hmpc)

xp = np.zeros(2)
iae_hmpc = 0.0
for k in range(200):
    hmpc.set_state(xp)
    u_vec = hmpc.compute_ref(xp, y_ref)
    u_s   = float(u_vec[0])
    xp_next = rk4_true(xp, np.array([u_s]), Ts)
    iae_hmpc += abs(xp[0] - 1.0) * Ts
    hmpc.add_state_observation(xp, np.array([u_s]), xp_next)
    xp = xp_next

print(f"  HybridMPC IAE={iae_hmpc:.4f}  observations={hmpc.observation_count()}")
print(f"  data model fitted: {hmpc.is_data_model_fitted()}")

# ---- Plain NonlinearMPC (no data model) ------------------------------------
model_plain = ctrl.HybridModel(f_phys, hmp, p_phys=p_phys)
nmpc_pars   = make_nmpc_params()
# Use the model's dynamicsFunc indirectly: wrap as a Python lambda
def plain_dynamics(x, u):
    return model_plain.predict(x, u)

nmpc = ctrl.NonlinearMPC(nmpc_pars, plain_dynamics)

xp = np.zeros(2)
iae_plain = 0.0
for k in range(200):
    nmpc.set_state(xp)
    u_vec = nmpc.compute_ref(xp, y_ref)
    xp = rk4_true(xp, np.array([float(u_vec[0])]), Ts)
    iae_plain += abs(xp[0] - 1.0) * Ts

print(f"  NonlinearMPC IAE={iae_plain:.4f}")
print(f"  IAE ratio (HybridMPC/NonlinearMPC) = {iae_hmpc/max(iae_plain, 1e-9):.3f}")

assert hmpc.is_data_model_fitted(),     "HybridMPC should fit data model"
assert np.isfinite(iae_hmpc),           "HybridMPC IAE not finite"
assert np.isfinite(iae_plain),          "NonlinearMPC IAE not finite"

print("\n[PASS] All H1/H2/H4 checks passed.")
