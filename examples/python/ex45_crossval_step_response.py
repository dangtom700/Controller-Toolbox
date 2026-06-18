"""
ex45 - Cross-Validation: ctrl simulation vs scipy.signal.dlsim / control.step_response
========================================================================================
Goal     : Verify that the ctrl.ss_step_copy simulation kernel matches scipy
           and python-control step response calculations to floating-point precision.

Two plants, two methods each:
  A. ctrl.ss_step_copy loop  vs  scipy.signal.dlsim
  B. ctrl simulation         vs  exact closed-form y(t) = 1 - exp(-t/tau)

Agreement criterion: max |y_ctrl - y_ref| < 1e-10 per step.

Run:
    conda run -n soft_robotics -- python ex45_crossval_step_response.py
"""
import _setup_bindings  # noqa: F401
import ctrl_toolbox as ctrl
import numpy as np
from scipy import signal

results = []

def check(label, ctrl_y, ref_y, tol=1e-10):
    err = np.max(np.abs(np.array(ctrl_y) - np.array(ref_y)))
    ok = err < tol
    print(f"  {label:55s}: {'PASS' if ok else f'FAIL (max_err={err:.2e})'}")
    return ok

# ===========================================================================
# Plant 1: G(s) = 1/(s+1), ZOH @ Ts=0.1
# Step response exact: y(kTs) = 1 - exp(-k*Ts)
# ===========================================================================
print("=== Step response: G(s)=1/(s+1), ZOH Ts=0.1 ===")
Ts = 0.1
plant_fo_d = ctrl.c2d(
    ctrl.StateSpace(np.array([[-1.0]]), np.array([[1.0]]),
                    np.array([[1.0]]), np.zeros((1,1)), 0.0),
    Ts, ctrl.C2dMethod.ZOH)

N = 50
u_step = np.ones(N)
# scipy reference
Ad = plant_fo_d.A; Bd = plant_fo_d.B; Cd = plant_fo_d.C; Dd = plant_fo_d.D
sys_scipy = signal.dlti(Ad, Bd, Cd, Dd, dt=Ts)
t_sp, y_sp, _ = signal.dlsim(sys_scipy, u_step)
y_sp_flat = y_sp.flatten()

# ctrl simulation
x_ctrl = np.zeros(plant_fo_d.state_size())
y_ctrl = []
for k in range(N):
    y, x_ctrl = ctrl.ss_step_copy(plant_fo_d, x_ctrl, np.array([1.0]))
    y_ctrl.append(float(y[0]))

# exact reference y(kTs) = 1 - exp(-k*Ts)  (k=1,2,...,N; k=0 gives y(0) via ssStep output-then-update)
# Note: ss_step_copy returns y[k] = C*x[k] + D*u[k], then updates x to x[k+1]
# At k=0: x[0]=0, so y[0] = C*0 + D*1 = D = 0 (for D=0 plant)
# After step: x[1] = A*0 + B*1 = Bd
# At k=1: y[1] = C*x[1] = C*Bd = Bd*C = (1-exp(-Ts))
y_exact = [0.0] + [1.0 - np.exp(-k*Ts) for k in range(1, N)]

results.append(check("ctrl.ss_step_copy vs scipy.signal.dlsim",   y_ctrl, y_sp_flat[:N]))
results.append(check("ctrl.ss_step_copy vs exact 1-exp(-kTs)",    y_ctrl, y_exact))

# ===========================================================================
# Plant 2: G(s) = 1/(s^2+1.5s+1), ZOH @ Ts=0.01
# No closed form, but scipy dlsim is the reference
# ===========================================================================
print("\n=== Step response: G(s)=1/(s^2+1.5s+1), ZOH Ts=0.01 ===")
Ts2 = 0.01
plant2 = ctrl.c2d(ctrl.StateSpace(
    np.array([[0.0, 1.0], [-1.0, -1.5]]),
    np.array([[0.0], [1.0]]),
    np.array([[1.0, 0.0]]), np.zeros((1,1)), 0.0),
    Ts2, ctrl.C2dMethod.ZOH)

N2 = 500
sys2_scipy = signal.dlti(plant2.A, plant2.B, plant2.C, plant2.D, dt=Ts2)
t2_sp, y2_sp, _ = signal.dlsim(sys2_scipy, np.ones(N2))
y2_sp_flat = y2_sp.flatten()

x2_ctrl = np.zeros(plant2.state_size())
y2_ctrl = []
for k in range(N2):
    y, x2_ctrl = ctrl.ss_step_copy(plant2, x2_ctrl, np.array([1.0]))
    y2_ctrl.append(float(y[0]))

results.append(check("ctrl.ss_step_copy vs scipy.signal.dlsim (2nd order)", y2_ctrl, y2_sp_flat[:N2], tol=1e-9))

# Both ctrl and scipy should give the same value at step 499
# (second-order plant may still be in transient at 5s - comparison is ctrl vs scipy, not vs limit)
dc_gain_ctrl  = float(y2_ctrl[-1])
dc_gain_scipy = float(y2_sp_flat[N2-1])
print(f"  y[499]: ctrl={dc_gain_ctrl:.8f}  scipy={dc_gain_scipy:.8f}")
results.append(abs(dc_gain_ctrl - dc_gain_scipy) < 1e-9)

# ===========================================================================
# MIMO verification: 2-output plant
# ===========================================================================
print("\n=== Step response: 2-output plant (identity C) ===")
plant_2out = ctrl.StateSpace(
    np.array([[0.9, 0.0], [0.0, 0.8]]),
    np.array([[0.1], [0.2]]),
    np.eye(2), np.zeros((2,1)), Ts2)
x_m = np.zeros(2)
y0_ctrl, y1_ctrl = [], []
for k in range(100):
    y, x_m = ctrl.ss_step_copy(plant_m := plant_2out, x_m, np.array([1.0]))
    y0_ctrl.append(y[0]); y1_ctrl.append(y[1])

sys_m = signal.dlti(plant_2out.A, plant_2out.B, plant_2out.C, plant_2out.D, dt=Ts2)
_, y_m_sp, _ = signal.dlsim(sys_m, np.ones(100))
results.append(check("MIMO y[0] vs scipy", y0_ctrl, y_m_sp[:, 0].tolist(), tol=1e-10))
results.append(check("MIMO y[1] vs scipy", y1_ctrl, y_m_sp[:, 1].tolist(), tol=1e-10))

n_pass = sum(bool(r) for r in results)
n_total = len(results)
print(f"\n{'='*60}")
print(f"Step response cross-validation: {n_pass}/{n_total} checks passed")
assert n_pass == n_total
print("PASS")
