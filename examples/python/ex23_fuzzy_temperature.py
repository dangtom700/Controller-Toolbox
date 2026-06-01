"""
ex23_fuzzy_temperature.py
Generate simulation data for the HVAC temperature control example
and produce a two-panel comparison plot.

Plant: G(s) = 1 / ((200s+1)(60s+1))  [degC / kW]
Controllers: FuzzyPD vs discrete PID
"""
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import cont2discrete
from pathlib import Path

# -- Plant ---------------------------------------------------------------------
Ts  = 5.0   # s
# Ac = [-1/200  0; 1/60  -1/60], Bc = [1/200; 0], Cc = [0 1]
Ac = np.array([[-1/200, 0], [1/60, -1/60]])
Bc = np.array([[1/200], [0]])
Cc = np.array([[0, 1]])
Dc = np.array([[0]])
sys_d = cont2discrete((Ac, Bc, Cc, Dc), Ts, method='zoh')
Ad, Bd, Cd, Dd = sys_d[:4]

def plant_step(x, u, A=Ad, B=Bd, C=Cd, D=Dd):
    xn = A @ x + B * float(u)
    y  = float((C @ xn).flat[0])
    return xn, y

# -- FuzzyPD (Mamdani, 5 terms, CoG) -----------------------------------------
def tri(a, c, b, x):
    if x <= a or x >= b: return 0.0
    if x <= c: return (x-a)/(c-a+1e-12)
    return (b-x)/(b-c+1e-12)

def shoulder_l(a, b, x):
    if x <= a: return 1.0
    if x >= b: return 0.0
    return (b-x)/(b-a+1e-12)

def shoulder_r(a, b, x):
    if x <= a: return 0.0
    if x >= b: return 1.0
    return (x-a)/(b-a+1e-12)

TERMS_5 = [
    lambda x: shoulder_l(-1.0, -0.5, x),    # NL
    lambda x: tri(-1.0, -0.5, 0.0, x),       # NS
    lambda x: tri(-0.5,  0.0, 0.5, x),       # ZE
    lambda x: tri( 0.0,  0.5, 1.0, x),       # PS
    lambda x: shoulder_r( 0.5,  1.0, x),     # PL
]

RULE_TABLE = [
    [0, 0, 0, 1, 2],
    [0, 0, 1, 2, 3],
    [0, 1, 2, 3, 4],
    [1, 2, 3, 4, 4],
    [2, 3, 4, 4, 4],
]

def fuzzy_pd(e, de, e_scale=2.0, de_scale=0.2, u_scale=3.0,
             u_min=0.0, u_max=3.0, n_pts=101):
    en  = float(np.clip(e  / e_scale,  -1, 1))
    den = float(np.clip(de / de_scale, -1, 1))
    mu_e  = [f(en)  for f in TERMS_5]
    mu_de = [f(den) for f in TERMS_5]

    strengths = np.zeros(5)
    for ie in range(5):
        for ide in range(5):
            w = mu_e[ie] * mu_de[ide]
            ct = RULE_TABLE[ie][ide]
            strengths[ct] = max(strengths[ct], w)

    # CoG over [-1, 1]
    xs = np.linspace(-1, 1, n_pts)
    agg = np.zeros(n_pts)
    for k, xk in enumerate(xs):
        vals = [min(strengths[t], TERMS_5[t](xk)) for t in range(5)]
        agg[k] = max(vals)
    denom = agg.sum()
    u_n = (xs * agg).sum() / (denom + 1e-12)
    return float(np.clip(u_n * u_scale, u_min, u_max))

# -- PID (backward Euler with anti-windup) ------------------------------------
class PID:
    def __init__(self, Kp, Ki, Kd, N, Kb, umin, umax, Ts):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.N, self.Kb = N, Kb
        self.umin, self.umax, self.Ts = umin, umax, Ts
        self.integ = 0.0; self.d_filt = 0.0; self.u_prev = 0.0

    def compute(self, e):
        P = self.Kp * e
        self.integ += self.Ki * self.Ts * e
        alpha = self.N * self.Ts / (1 + self.N * self.Ts)
        self.d_filt = (1-alpha)*self.d_filt + alpha*self.Kd*self.N*e
        u_unsat = P + self.integ + self.d_filt
        u_sat   = float(np.clip(u_unsat, self.umin, self.umax))
        self.integ += self.Kb * (u_sat - u_unsat)
        self.u_prev = u_sat
        return u_sat

pid = PID(1.8, 0.006, 40.0, 5.0, 1.0, 0.0, 3.0, Ts)

# -- Simulation ----------------------------------------------------------------
N   = 200
ref = 22.0
# Steady-state at y=20 with u=0: x_ss = -(A-I)^-1 * B * 0 = 0, so just start at 0
# and let the controller drive from 20->22.  Offset y by 20 as initial output.
yf  = 0.0; xf = np.zeros((2, 1))
yp  = 0.0; xp = np.zeros((2, 1))
# Represent temperature as delta from 20^\circC; setpoint becomes ref=2
ref_delta = 2.0
e_prev_f  = 0.0

ts, refs, yfs, ufs, yps, ups, dists = [], [], [], [], [], [], []
for k in range(N):
    t    = k * Ts
    dist = -0.5 if t >= 300 else 0.0

    de_f = (ref_delta - yf - e_prev_f) / Ts
    uf   = fuzzy_pd(ref_delta - yf, de_f)
    uf   = float(np.clip(uf + dist, 0, 3))
    e_prev_f = ref_delta - yf

    up = pid.compute(ref_delta - yp)
    up = float(np.clip(up + dist, 0, 3))

    xf, yf = plant_step(xf, uf)
    xp, yp = plant_step(xp, up)

    # Convert back to absolute temperature for output
    y_abs_f = yf + 20.0
    y_abs_p = yp + 20.0

    ts.append(t); refs.append(ref)
    yfs.append(y_abs_f); ufs.append(uf)
    yps.append(y_abs_p); ups.append(up)
    dists.append(0 if dist == 0 else 1)

ts = np.array(ts)

# -- Save CSV -----------------------------------------------------------------
out = Path(__file__).parent.parent.parent / "data" / "ex23_fuzzy_temperature.csv"
header = "t,ref,y_fuzzy,u_fuzzy,y_pid,u_pid,disturbance"
data = np.column_stack([ts, refs, yfs, ufs, yps, ups, dists])
np.savetxt(out, data, delimiter=",", header=header, comments="", fmt="%.4f")
print(f"Saved: {out}")

# -- Plot ---------------------------------------------------------------------
fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
ax1, ax2 = axes

ax1.plot(ts, refs, 'k--', lw=1.2, label='Reference')
ax1.plot(ts, yfs,  'b',   lw=1.5, label='FuzzyPD')
ax1.plot(ts, yps,  'r--', lw=1.5, label='PID')
ax1.axvspan(300, ts[-1], alpha=0.08, color='orange', label='Disturbance')
ax1.set_ylabel('Temperature [^\\circC]')
ax1.legend(); ax1.grid(True, alpha=0.4)
ax1.set_title('HVAC Temperature Control - FuzzyPD vs PID')

ax2.plot(ts, ufs, 'b',   lw=1.5, label='FuzzyPD output')
ax2.plot(ts, ups, 'r--', lw=1.5, label='PID output')
ax2.set_ylabel('Heater power [kW]')
ax2.set_xlabel('Time [s]')
ax2.legend(); ax2.grid(True, alpha=0.4)

plt.tight_layout()
fig.savefig(Path(__file__).parent.parent.parent / "data" / "ex23_fuzzy_temperature.png", dpi=150)
plt.show(block=False)
plt.pause(10)
plt.close('all')
