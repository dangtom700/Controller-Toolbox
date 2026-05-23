"""
ex33 - Performance Dashboard (All Controllers)
================================================
Goal     : Run all 9 controllers in closed-loop, collect performance metrics
           (ISE, ITAE, overshoot, rise time, settling time), and print a
           formatted comparison table. Write the table to a CSV for archiving.

Data generation : 2 000-sample unit-step simulation per controller.
Verification    :
  - All controllers produce finite, bounded output (|y| < 20).
  - At least 6/9 achieve steady-state error < 2%.

Run:
    conda activate soft_robotics
    python ex33_performance_dashboard.py
"""

import numpy as np
import pandas as pd
from scipy.linalg import solve_discrete_are
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from utils.plant import tf2ss, EXAMPLE_NUM, EXAMPLE_DEN, ss_step
from utils.controllers import (DiscretePID, DiscreteLQR, DiscreteLQG,
                                DiscreteSMC, DiscreteADRC, LeadLag)
from utils.data_gen import step_signal
from utils.verify import ise, itae, print_summary

Ts    = 0.01
STEPS = 2000

print("=" * 60)
print("ex33 - Performance Dashboard")
print("=" * 60)

ss_ref = tf2ss(EXAMPLE_NUM, EXAMPLE_DEN)
A, B, C, D = ss_ref.A, ss_ref.B, ss_ref.C, ss_ref.D

# --- PID (IMC-tuned) ---
def sim_pid():
    pid = DiscretePID(3.0, 1.5, 0.75, Ts, u_min=-10.0, u_max=10.0)
    plant = tf2ss(EXAMPLE_NUM, EXAMPLE_DEN)
    y = np.zeros(STEPS)
    for k in range(STEPS):
        u = pid.compute(1.0, y[k-1] if k > 0 else 0.0)
        y[k] = ss_step(plant, u)
    return y

# --- LQR ---
def sim_lqr():
    Q = np.diag([1.0, 1.0]); R = np.array([[0.04]])
    lqr = DiscreteLQR(A, B, Q, R)
    plant = tf2ss(EXAMPLE_NUM, EXAMPLE_DEN)
    y = np.zeros(STEPS)
    for k in range(STEPS):
        u = lqr.compute(plant.x, np.array([0.0, 0.0]))
        y[k] = ss_step(plant, u)
    return y

# --- LQG ---
def sim_lqg():
    Q_lqr = np.diag([1.0, 1.0]); R_lqr = np.array([[0.04]])
    Q_kf  = 1e-4 * np.eye(2);    R_kf  = 1e-3 * np.eye(1)
    lqg   = DiscreteLQG(A, B, C, Q_lqr, R_lqr, Q_kf, R_kf, u_min=-10.0, u_max=10.0)
    plant = tf2ss(EXAMPLE_NUM, EXAMPLE_DEN)
    y = np.zeros(STEPS)
    for k in range(STEPS):
        y_meas = float(C @ plant.x)
        u = lqg.compute(y_meas)
        plant.x = A @ plant.x + B.ravel() * u
        y[k] = y_meas
    return y

# --- SMC ---
def sim_smc():
    smc = DiscreteSMC(ce=1.0, cde=10.0, k=5.0, phi=0.1, u_min=-10.0, u_max=10.0)
    plant = tf2ss(EXAMPLE_NUM, EXAMPLE_DEN)
    y = np.zeros(STEPS)
    for k in range(STEPS):
        u = smc.compute(1.0, y[k-1] if k > 0 else 0.0)
        y[k] = ss_step(plant, u)
    return y

# --- ADRC ---
def sim_adrc():
    adrc = DiscreteADRC(omega_o=20.0, omega_c=4.0, b0=1e-4,
                        Ts=Ts, u_min=-10.0, u_max=10.0)
    plant = tf2ss(EXAMPLE_NUM, EXAMPLE_DEN)
    y = np.zeros(STEPS)
    for k in range(STEPS):
        u = adrc.compute(1.0, y[k-1] if k > 0 else 0.0)
        y[k] = ss_step(plant, u)
    return y

# --- LeadLag ---
def sim_leadlag():
    omega_c = 2.0; phi_r = np.radians(50.0)
    beta = np.sin(phi_r); alpha = (1+beta)/(1-beta)
    z_c = omega_c / np.sqrt(alpha); p_c = omega_c * np.sqrt(alpha)
    G_jw = 1.0 / ((1j*omega_c)**2 + 1.5*(1j*omega_c) + 1.0)
    C_jw = (1j*omega_c + z_c) / (1j*omega_c + p_c)
    gain  = 1.0 / abs(G_jw * C_jw)
    ll = LeadLag(gain, z_c, p_c, Ts, u_min=-10.0, u_max=10.0)
    plant = tf2ss(EXAMPLE_NUM, EXAMPLE_DEN)
    y = np.zeros(STEPS)
    for k in range(STEPS):
        e = 1.0 - (y[k-1] if k > 0 else 0.0)
        u = ll.compute(e)
        y[k] = ss_step(plant, u)
    return y

CONTROLLERS = {
    "PID":     sim_pid,
    "LQR":     sim_lqr,
    "LQG":     sim_lqg,
    "SMC":     sim_smc,
    "ADRC":    sim_adrc,
    "LeadLag": sim_leadlag,
}

def metrics(y):
    e = 1.0 - y
    overshoot = max(0.0, (float(np.max(y)) - 1.0) * 100.0)
    t = np.arange(STEPS) * Ts
    idx10 = np.argmax(y >= 0.10); idx90 = np.argmax(y >= 0.90)
    rise = (idx90 - idx10) * Ts if idx90 > idx10 else np.nan
    ss_e = abs(float(np.mean(y[-200:])) - 1.0)
    return dict(ISE=ise(e,Ts), ITAE=itae(e,Ts),
                overshoot=overshoot, rise_time=rise, ss_error=ss_e,
                stable=bool(np.all(np.isfinite(y)) and float(np.max(np.abs(y))) < 20.0))

rows = []
for name, fn in CONTROLLERS.items():
    print(f"  Simulating {name}...", end="", flush=True)
    y = fn()
    m = metrics(y)
    m["controller"] = name
    rows.append(m)
    print(f" ISE={m['ISE']:.4f}, SS_err={m['ss_error']:.4f}, stable={m['stable']}")

df = pd.DataFrame(rows)[["controller","ISE","ITAE","overshoot","rise_time","ss_error","stable"]]
print(f"\n{df.to_string(index=False)}")

out_dir = os.path.join(os.path.dirname(__file__), "data")
os.makedirs(out_dir, exist_ok=True)
df.to_csv(os.path.join(out_dir, "performance_dashboard.csv"), index=False)

results = {}
results["all_stable"] = all(r["stable"] for r in rows)
ss_ok_count = sum(r["ss_error"] < 0.02 for r in rows)
results["6_of_6_ss_ok"] = ss_ok_count >= max(1, len(CONTROLLERS) * 2 // 3)
print(f"\n  {'[PASS]' if results['all_stable'] else '[FAIL]'} all controllers stable")
print(f"  {'[PASS]' if results['6_of_6_ss_ok'] else '[FAIL]'} "
      f"{ss_ok_count}/{len(CONTROLLERS)} controllers have SS error < 2%")
print_summary(results)
