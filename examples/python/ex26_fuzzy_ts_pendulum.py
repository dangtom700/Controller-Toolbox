"""
ex26_fuzzy_ts_pendulum.py
Generate simulation data and plots for the inverted pendulum gain-scheduling example.

Plant: true nonlinear inverted pendulum (Euler integration)
  theta_ddot = (g/l)*sin(theta) - (b/ml^2)*theta_dot + (1/ml^2)*F

TS Fuzzy Gain Scheduler blends two PIDs tuned at different operating points.
"""
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# -- Pendulum parameters -------------------------------------------------------
g=9.81; l=0.5; m=0.1; b_fric=0.05

def pend_step(theta, theta_dot, F, dt):
    alpha = (g/l)*np.sin(theta) - (b_fric/(m*l**2))*theta_dot + (1/(m*l**2))*F
    theta_dot_n = theta_dot + alpha*dt
    theta_n     = theta + theta_dot_n*dt
    return theta_n, theta_dot_n

# -- PID (backward Euler) ------------------------------------------------------
class PID:
    def __init__(self, Kp, Ki, Kd, N, umin, umax, Ts):
        self.Kp,self.Ki,self.Kd,self.N=Kp,Ki,Kd,N
        self.umin,self.umax,self.Ts=umin,umax,Ts
        self.integ=0.0; self.d_filt=0.0; self.e_prev=0.0

    def compute(self, e):
        P = self.Kp*e
        self.integ += self.Ki*self.Ts*e
        a = self.N*self.Ts/(1+self.N*self.Ts)
        self.d_filt = (1-a)*self.d_filt + a*self.Kd*self.N*e
        u = float(np.clip(P+self.integ+self.d_filt, self.umin, self.umax))
        return u

Ts = 0.005

# -- TS weights using Gaussian MFs --------------------------------------------
def gauss(mean, sigma, x): return np.exp(-0.5*((x-mean)/(sigma+1e-12))**2)

def ts_weights(abs_theta):
    w1 = gauss(0.00, 0.15, abs_theta)
    w2 = gauss(0.35, 0.15, abs_theta)
    s  = w1 + w2 + 1e-12
    return w1/s, w2/s

# -- Controllers ---------------------------------------------------------------
pid1 = PID(18.0, 5.0, 2.5, 80.0, -15, 15, Ts)   # upright
pid2 = PID(40.0, 2.0, 5.0, 80.0, -15, 15, Ts)   # large-angle recovery
pid_base = PID(18.0, 5.0, 2.5, 80.0, -15, 15, Ts)

# -- Separate PID instances for independent compute calls ---------------------
# (pid1 and pid2 states must not be shared between w-scaled calls)
pid1b = PID(18.0, 5.0, 2.5, 80.0, -15, 15, Ts)
pid2b = PID(40.0, 2.0, 5.0, 80.0, -15, 15, Ts)

theta0 = 0.4   # rad - large initial tilt
theta_ts, dtheta_ts = theta0, 0.0
theta_base, dtheta_base = theta0, 0.0

rows = []
N = 2000   # 10 s

for k in range(N):
    t = k*Ts

    # TS scheduled
    w1, w2 = ts_weights(abs(theta_ts))
    e_ts = -theta_ts   # reference = 0
    u1 = pid1.compute(e_ts)
    u2 = pid2.compute(e_ts)
    u_ts = float(np.clip(w1*u1 + w2*u2, -15, 15))
    theta_ts, dtheta_ts = pend_step(theta_ts, dtheta_ts, u_ts, Ts)

    # Baseline
    u_base = pid_base.compute(-theta_base)
    u_base = float(np.clip(u_base, -15, 15))
    theta_base, dtheta_base = pend_step(theta_base, dtheta_base, u_base, Ts)

    rows.append([t, theta_ts, dtheta_ts, u_ts, theta_base, u_base, w1, w2])

rows = np.array(rows)
out = Path(__file__).parent.parent.parent / "data" / "ex26_fuzzy_ts_pendulum.csv"
np.savetxt(out, rows, delimiter=",",
           header="t,theta_ts,theta_dot_ts,u_ts,theta_base,u_base,w1,w2",
           comments="", fmt="%.5f")
print(f"Saved: {out}")

# -- Plot ---------------------------------------------------------------------
ts = rows[:,0]
fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)

axes[0].plot(ts, np.degrees(rows[:,1]), 'b',   lw=1.5, label='TS FuzzyPID')
axes[0].plot(ts, np.degrees(rows[:,4]), 'r--', lw=1.5, label='Single PID')
axes[0].axhline(0, color='k', ls=':', lw=1)
axes[0].set_ylabel('Angle [deg]')
axes[0].legend(); axes[0].grid(True, alpha=0.4)
axes[0].set_title('Inverted Pendulum - TS Fuzzy Gain Scheduler vs Fixed PID (theta0 = 0.4 rad)')

axes[1].plot(ts, rows[:,3], 'b',   lw=1.5, label='TS FuzzyPID')
axes[1].plot(ts, rows[:,5], 'r--', lw=1.5, label='Single PID')
axes[1].set_ylabel('Force F [N]')
axes[1].legend(); axes[1].grid(True, alpha=0.4)

axes[2].plot(ts, rows[:,6], 'b',   lw=1.5, label='w1 (upright PID)')
axes[2].plot(ts, rows[:,7], 'orange', lw=1.5, label='w2 (recovery PID)')
axes[2].set_ylabel('TS weight')
axes[2].set_xlabel('Time [s]')
axes[2].legend(); axes[2].grid(True, alpha=0.4)

plt.tight_layout()
fig.savefig(Path(__file__).parent.parent.parent / "data" / "ex26_fuzzy_ts_pendulum.png", dpi=150)
plt.show()
