"""
ex24_fuzzy_dc_motor.py
Generate simulation data and plots for the DC motor speed control example.

Plant: armature-controlled DC motor
  G(s) = 100 / (s(0.05s + 1))   [rad/s / V]
  Discretised at Ts = 0.002 s (ZOH).

FuzzyPID vs DiscretePID - speed ramp tracking with voltage saturation (+/-12 V).
"""
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import cont2discrete, lti, dlsim
from pathlib import Path

# -- Plant: integrating second-order ------------------------------------------
Ts   = 0.002
Km   = 100.0
tau  = 0.05
# G(s) = Km / (s * (tau*s + 1))
# State: x1 = omega (output), x2 = d(omega)/dt
Ac = np.array([[0,  1],
               [0, -1/tau]])
Bc = np.array([[0],
               [Km/tau]])
Cc = np.array([[1, 0]])
Dc = np.array([[0]])
Ad, Bd, Cd, Dd, _ = cont2discrete((Ac, Bc, Cc, Dc), Ts, method='zoh')

def plant_step(x, u):
    xn = Ad @ x + Bd * u
    y  = (Cd @ xn)[0, 0]
    return xn, y

U_MAX = 12.0

# -- FuzzyPD kernel (same engine as ex23) -------------------------------------
def tri(a, c, b, x):
    if x <= a or x >= b: return 0.0
    return (x-a)/(c-a+1e-12) if x <= c else (b-x)/(b-c+1e-12)

def shoulder_l(a, b, x):
    return 1.0 if x <= a else (0.0 if x >= b else (b-x)/(b-a+1e-12))

def shoulder_r(a, b, x):
    return 0.0 if x <= a else (1.0 if x >= b else (x-a)/(b-a+1e-12))

TERMS5 = [
    lambda x: shoulder_l(-1.0, -0.5, x),
    lambda x: tri(-1.0, -0.5, 0.0, x),
    lambda x: tri(-0.5,  0.0, 0.5, x),
    lambda x: tri( 0.0,  0.5, 1.0, x),
    lambda x: shoulder_r( 0.5,  1.0, x),
]
RULE = [[0,0,0,1,2],[0,0,1,2,3],[0,1,2,3,4],[1,2,3,4,4],[2,3,4,4,4]]

def cog_defuzz(strengths, n=101):
    xs = np.linspace(-1, 1, n)
    agg = np.array([max(min(strengths[t], TERMS5[t](xk)) for t in range(5)) for xk in xs])
    den = agg.sum()
    return float((xs * agg).sum() / (den + 1e-12))

def fuzzy_pd_raw(e_n, de_n):
    mu_e  = [f(e_n)  for f in TERMS5]
    mu_de = [f(de_n) for f in TERMS5]
    strengths = np.zeros(5)
    for ie in range(5):
        for ide in range(5):
            ct = RULE[ie][ide]
            strengths[ct] = max(strengths[ct], mu_e[ie]*mu_de[ide])
    return cog_defuzz(strengths)

class FuzzyPID:
    def __init__(self, e_sc, de_sc, u_sc, Ki, Kb, umin, umax, Ts):
        self.e_sc, self.de_sc, self.u_sc = e_sc, de_sc, u_sc
        self.Ki, self.Kb = Ki, Kb
        self.umin, self.umax, self.Ts = umin, umax, Ts
        self.integ = 0.0; self.e_prev = 0.0; self.u_prev = 0.0

    def compute(self, e):
        de  = (e - self.e_prev) / self.Ts
        e_n = np.clip(e  / self.e_sc,  -1, 1)
        de_n= np.clip(de / self.de_sc, -1, 1)
        u_pd = fuzzy_pd_raw(e_n, de_n) * self.u_sc
        ki_upd    = self.Ki * self.Ts * e
        u_unsat   = u_pd + self.integ + ki_upd
        u_sat     = float(np.clip(u_unsat, self.umin, self.umax))
        self.integ += ki_upd + self.Kb * (u_sat - u_unsat)
        self.e_prev = e; self.u_prev = u_sat
        return u_sat

class PID:
    def __init__(self, Kp, Ki, Kd, N, Kb, umin, umax, Ts):
        self.Kp,self.Ki,self.Kd,self.N,self.Kb=Kp,Ki,Kd,N,Kb
        self.umin,self.umax,self.Ts=umin,umax,Ts
        self.integ=0.0; self.d_filt=0.0; self.e_prev=0.0

    def compute(self, e):
        P = self.Kp * e
        self.integ += self.Ki * self.Ts * e
        a = self.N*self.Ts/(1+self.N*self.Ts)
        self.d_filt = (1-a)*self.d_filt + a*self.Kd*self.N*e
        u_unsat = P + self.integ + self.d_filt
        u_sat   = float(np.clip(u_unsat, self.umin, self.umax))
        self.integ += self.Kb*(u_sat - u_unsat)
        return u_sat

fuzzy = FuzzyPID(30.0, 500.0, 10.0, 2.0, 0.8, -U_MAX, U_MAX, Ts)
pid   = PID(0.18, 3.6, 0.001, 50.0, 0.8, -U_MAX, U_MAX, Ts)

def ref_fn(t):
    if t < 0.5:  return 0.0
    if t < 1.0:  return 100*(t-0.5)/0.5
    if t < 2.5:  return 100.0
    if t < 3.0:  return 100-100*(t-2.5)/0.5
    return 50.0

N    = 2500
xf   = np.zeros((2,1)); xp = np.zeros((2,1))
yf   = 0.0; yp = 0.0

rows = []
for k in range(N):
    t = k*Ts; r = ref_fn(t)
    uf = fuzzy.compute(r - yf)
    up = pid.compute(r - yp)
    sat_f = int(abs(uf) >= U_MAX - 1e-6)
    sat_p = int(abs(up) >= U_MAX - 1e-6)
    xf, yf = plant_step(xf, uf)
    xp, yp = plant_step(xp, up)
    rows.append([t, r, yf, uf, yp, up, sat_f, sat_p])

rows = np.array(rows)
out = Path(__file__).parent.parent / "data" / "ex24_fuzzy_dc_motor.csv"
np.savetxt(out, rows, delimiter=",",
           header="t,ref,y_fuzzy,u_fuzzy,y_pid,u_pid,sat_fuzzy,sat_pid",
           comments="", fmt="%.5f")
print(f"Saved: {out}")

# -- Plot ---------------------------------------------------------------------
fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
ts = rows[:,0]

axes[0].plot(ts, rows[:,1], 'k--', lw=1.2, label='Reference')
axes[0].plot(ts, rows[:,2], 'b',   lw=1.5, label='FuzzyPID')
axes[0].plot(ts, rows[:,4], 'r--', lw=1.5, label='PID')
axes[0].set_ylabel('Speed [rad/s]')
axes[0].legend(); axes[0].grid(True, alpha=0.4)
axes[0].set_title('DC Motor Speed Control - FuzzyPID vs PID')

axes[1].plot(ts, rows[:,3], 'b',   lw=1.5, label='FuzzyPID')
axes[1].plot(ts, rows[:,5], 'r--', lw=1.5, label='PID')
axes[1].axhline( U_MAX, color='k', ls=':', lw=1)
axes[1].axhline(-U_MAX, color='k', ls=':', lw=1)
axes[1].set_ylabel('Voltage [V]')
axes[1].legend(); axes[1].grid(True, alpha=0.4)

axes[2].fill_between(ts, rows[:,6], alpha=0.5, color='b', label='FuzzyPID sat')
axes[2].fill_between(ts, rows[:,7], alpha=0.5, color='r', label='PID sat')
axes[2].set_ylabel('Saturation (0/1)')
axes[2].set_xlabel('Time [s]')
axes[2].legend(); axes[2].grid(True, alpha=0.4)

plt.tight_layout()
fig.savefig(Path(__file__).parent.parent / "data" / "ex24_fuzzy_dc_motor.png", dpi=150)
plt.show()
