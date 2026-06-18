"""
ex41 - EKF State Estimation: Nonlinear Pendulum (ctrl_toolbox bindings)
========================================================================
Goal     : Demonstrate ExtendedKalmanFilter with nonlinear dynamics.
           Uses analytical Jacobians to track pendulum state from noisy
           angle-only measurements.

Scenario : Simple pendulum (L=1m, m=1kg, b=0.1).  Only theta measured
           (sigma = 5 deg).  EKF linearises at each step to recover
           both theta and theta_dot.  PD control drives to theta=0.

Note     : Requires CTRL_HAS_ADVANCED_KALMAN (enabled by default).

Run:
    conda run -n soft_robotics -- python ex41_ekf_nonlinear_pendulum.py
"""
import _setup_bindings  # noqa: F401
import ctrl_toolbox as ctrl
import numpy as np

feats = ctrl.features()
if not feats.get('advanced_kalman', False):
    print("Skipping: advanced_kalman not compiled in.")
    raise SystemExit(0)

rng = np.random.default_rng(7)
g=9.81; L=1.0; m=1.0; b=0.1; Ts=0.02
sig_meas = 5*np.pi/180   # 5 degrees

def f(x, u):
    th, dth = x[0], x[1]
    ddth = -(g/L)*np.sin(th) - (b/m)*dth + float(u[0])/(m*L**2)
    return np.array([th+Ts*dth, dth+Ts*ddth])

def h(x, u): return np.array([x[0]])
def Fj(x, u): return np.array([[1.0,Ts],[-(g/L)*np.cos(x[0])*Ts,1-(b/m)*Ts]])
def Hj(x, u): return np.array([[1.0, 0.0]])

ekf = ctrl.ExtendedKalmanFilter(n=2,p=1,f=f,h=h,F_jac=Fj,H_jac=Hj,
    Q=np.diag([1e-5,1e-4]), R=np.array([[sig_meas**2]]), Ts=Ts)

x_true=np.array([0.5, 0.0]); ekf.set_state(np.array([0.4, 0.0]))
u_prev=np.zeros(1); est_errs=[]

for k in range(int(5.0/Ts)):
    y_n = np.array([x_true[0]+rng.normal(0,sig_meas)])
    ekf.step(y_n, u_prev)
    est_errs.append(ekf.state()[0]-x_true[0])
    u = np.clip(np.array([10*(0-ekf.state()[0])-2*ekf.state()[1]]), -10.0, 10.0)
    x_true = f(x_true, u)
    u_prev = u

rmse = np.sqrt(np.mean(np.array(est_errs)**2))
print("=== EKF Nonlinear Pendulum ===")
print(f"Meas noise sigma: {sig_meas*180/np.pi:.1f} deg")
print(f"EKF RMSE:         {rmse*180/np.pi:.3f} deg  ({100*rmse/sig_meas:.1f}% of noise)")
assert rmse < sig_meas*0.5, "EKF should beat raw measurement noise"
print("PASS")
