"""
ex25_fuzzy_supervisor_mpc.py
Generate simulation data and plots for the pH neutralisation example.

Plant: Hammerstein model
  Linear:  G(s) = 1 / (30s + 1)
  NL gain: k_pH(y) = 5 - 3*tanh(2*(y - 7))

Fixed MPC vs FuzzySupervisor-driven adaptive MPC.
"""
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import cont2discrete
from pathlib import Path

Ts = 2.0

# -- Nonlinear gain ------------------------------------------------------------
def ph_gain(y): return 5.0 - 3.0*np.tanh(2.0*(y - 7.0))

# -- Linear first-order plant (scaled by k) ------------------------------------
def build_plant(k):
    a = np.exp(-Ts/30.0)
    b = k*(1-a)
    return a, b   # y[k+1] = a*y[k] + b*u[k]

# -- Simple unconstrained SISO MPC (batch QP, no constraints for Python demo) --
class MPC:
    def __init__(self, a, b, Np=20, Nc=5, rho_y=10.0, rho_u=0.01):
        self.Np, self.Nc = Np, Nc
        self.rho_y, self.rho_u = rho_y, rho_u
        self.a, self.b = a, b
        self.u_prev = 0.0
        self._rebuild()

    def _rebuild(self):
        a, b, Np, Nc = self.a, self.b, self.Np, self.Nc
        F = np.array([[a**(i+1)] for i in range(Np)])
        Phi = np.zeros((Np, Nc))
        for i in range(Np):
            for j in range(min(i+1, Nc)):
                Phi[i,j] = (a**(i-j)) * b
        Q = self.rho_y * np.eye(Np)
        R = self.rho_u * np.eye(Nc)
        H = Phi.T @ Q @ Phi + R
        self.H_inv_PhiQ = np.linalg.solve(H, Phi.T @ Q)
        self.F = F

    def set_plant(self, a, b):
        self.a, self.b = a, b
        self._rebuild()

    def compute(self, x, r):
        err = r - x
        # prediction error vector
        free_resp = self.F * x
        R_vec = np.full((self.Np, 1), r)
        dU = -self.H_inv_PhiQ @ (free_resp - R_vec)
        u  = self.u_prev + float(dU[0, 0])
        u  = float(np.clip(u, 0, 100))
        self.u_prev = u
        return u

# -- Fuzzy Supervisor (Python re-implementation matching FuzzyLogic.h) ---------
def gauss(mean, sigma, x): return np.exp(-0.5*((x-mean)/(sigma+1e-12))**2)

def trap(a, b, c, d, x):
    if x <= a or x >= d: return 0.0
    if b <= x <= c:       return 1.0
    return (x-a)/(b-a+1e-12) if x < b else (d-x)/(d-c+1e-12)

def tri(a, c, b, x):
    if x <= a or x >= b: return 0.0
    return (x-a)/(c-a+1e-12) if x <= c else (b-x)/(b-c+1e-12)

def sh_r(a, b, x): return 0.0 if x<=a else (1.0 if x>=b else (x-a)/(b-a+1e-12))

def supervisor_signal(err_n, trend_n):
    # Input 1: error_norm [0, 1.5] - 3 terms
    mu_e = [
        trap(0, 0, 0.25, 0.45, err_n),                    # Small
        tri(0.30, 0.50, 0.70, err_n),                      # Medium
        sh_r(0.60, 0.90, err_n),                           # Large
    ]
    # Input 2: trend [-1, 1] - 3 terms
    mu_t = [
        sh_r(-1.5, -0.1, -trend_n),                        # Decreasing (mirror)
        tri(-0.3, 0.0, 0.3, trend_n),                      # Steady
        sh_r(0.1, 0.6, trend_n),                           # Increasing
    ]
    # Output terms: None=0, Maybe=0.5, High=0.8, Full=1.0
    out_centres = [0.0, 0.5, 0.8, 1.0]
    table = [[0,0,1],[0,1,2],[1,2,3]]
    strengths = np.zeros(4)
    for ie in range(3):
        for it in range(3):
            w = mu_e[ie]*mu_t[it]
            ct = table[ie][it]
            strengths[ct] = max(strengths[ct], w)
    # Weighted average defuzz
    num = sum(strengths[t]*out_centres[t] for t in range(4))
    den = sum(strengths) + 1e-12
    return float(num/den)

# -- Simulation ----------------------------------------------------------------
def ref_fn(t):
    if t <  60: return 7.0
    if t < 180: return 9.0
    if t < 300: return 5.0
    return 8.0

k_nom = ph_gain(7.0)
mpc_f = MPC(*build_plant(k_nom))
mpc_a = MPC(*build_plant(k_nom))

yf = 7.0; ya = 7.0
err_prev = 0.0
cooldown = 0; relinearise_count = 0
SIGNAL_THRESHOLD = 0.5
COOLDOWN_STEPS   = 15
E_THRESH = 0.8; TREND_THRESH = 0.05

rows = []
N = 240
for k in range(N):
    t = k*Ts; r = ref_fn(t)

    # Fixed MPC
    uf = mpc_f.compute(yf, r)
    k_t_f = ph_gain(yf)
    a_f, b_f = build_plant(k_t_f)
    yf = a_f*yf + b_f*uf

    # Adaptive MPC
    ua = mpc_a.compute(ya, r)
    ea = abs(r - ya)
    de = (ea - err_prev)/(Ts+1e-12)
    err_n  = ea / (E_THRESH+1e-12)
    trend_n= np.clip(de / (TREND_THRESH+1e-12), -1, 1)
    sig    = supervisor_signal(np.clip(err_n, 0, 1.5), trend_n)
    err_prev = ea

    did_relin = False
    if cooldown > 0:
        cooldown -= 1
    elif sig > SIGNAL_THRESHOLD:
        k_op = ph_gain(ya)
        mpc_a.set_plant(*build_plant(k_op))
        cooldown = COOLDOWN_STEPS
        relinearise_count += 1
        did_relin = True

    k_t_a = ph_gain(ya)
    a_a, b_a = build_plant(k_t_a)
    ya = a_a*ya + b_a*ua

    rows.append([t, r, yf, uf, ya, ua, sig, int(did_relin), k_t_a])

rows = np.array(rows)
out = Path(__file__).parent.parent.parent / "data" / "ex25_fuzzy_supervisor_mpc.csv"
np.savetxt(out, rows, delimiter=",",
           header="t,ref,y_mpc_fixed,u_mpc_fixed,y_mpc_adapt,u_mpc_adapt,"
                  "relinearize_signal,relinearize_event,k_eff",
           comments="", fmt="%.4f")
print(f"Saved: {out}  |  total re-linearisations: {relinearise_count}")

# -- Plot ---------------------------------------------------------------------
fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
ts = rows[:,0]

axes[0].plot(ts, rows[:,1], 'k--', lw=1.2, label='Reference')
axes[0].plot(ts, rows[:,2], 'r',   lw=1.5, label='MPC fixed')
axes[0].plot(ts, rows[:,4], 'b',   lw=1.5, label='MPC adaptive')
axes[0].set_ylabel('pH'); axes[0].legend(); axes[0].grid(True, alpha=0.4)
axes[0].set_title('pH Neutralisation - Fixed vs Adaptive (FuzzySupervisor) MPC')

axes[1].plot(ts, rows[:,3], 'r',   lw=1.5, label='Fixed')
axes[1].plot(ts, rows[:,5], 'b',   lw=1.5, label='Adaptive')
axes[1].set_ylabel('Valve [%]'); axes[1].legend(); axes[1].grid(True, alpha=0.4)

axes[2].plot(ts, rows[:,6], 'purple', lw=1.5, label='Supervisor signal')
axes[2].axhline(SIGNAL_THRESHOLD, color='k', ls=':', lw=1, label='Threshold')
evts = ts[rows[:,7].astype(bool)]
for ev in evts:
    axes[2].axvline(ev, color='orange', alpha=0.8, lw=1.2)
axes[2].set_ylabel('Signal [0-1]'); axes[2].set_xlabel('Time [s]')
axes[2].legend(); axes[2].grid(True, alpha=0.4)

plt.tight_layout()
fig.savefig(Path(__file__).parent.parent.parent / "data" / "ex25_fuzzy_supervisor_mpc.png", dpi=150)
plt.show()
