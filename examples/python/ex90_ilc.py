"""
ex90_ilc.py -- Iterative Learning Control (ILC) demo.

Demonstrates P-type and norm-optimal ILC on a double-integrator plant that
repeats a sinusoidal reference over multiple trials.  Shows how RMS error
converges across trials compared to a pure PID baseline.
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'build', 'bindings'))
if sys.platform == "win32" and hasattr(os, "add_dll_directory"):
    for _p in [r"C:\msys64\mingw64\bin"]:
        if os.path.isdir(_p):
            os.add_dll_directory(_p)

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'ILC'):
        raise AttributeError("ILC not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

import numpy as np

TS = 0.01
N  = 200  # trial length
N_TRIALS = 25


# ---------------------------------------------------------------------------
# Double-integrator plant
# ---------------------------------------------------------------------------
class Plant:
    def __init__(self):
        self.pos = 0.0
        self.vel = 0.0

    def step(self, u):
        y = self.pos
        self.vel += TS * u
        self.pos += TS * self.vel
        return y

    def reset(self):
        self.pos = self.vel = 0.0


# ---------------------------------------------------------------------------
# Reference: one period of a sine wave
# ---------------------------------------------------------------------------
t = np.arange(N) * TS
r = np.sin(2 * np.pi * t / (N * TS))


# ---------------------------------------------------------------------------
# Markov matrix for norm-optimal ILC
# G[i,j] = TS^2 * (i-j+1) for i>=j, else 0  (double integrator)
# ---------------------------------------------------------------------------
G = np.zeros((N, N))
for i in range(N):
    for j in range(i + 1):
        G[i, j] = TS**2 * (i - j + 1)
G_eigen = G  # numpy array; passed as Eigen MatrixXd in binding


# ---------------------------------------------------------------------------
# PID helper
# ---------------------------------------------------------------------------
def make_pid():
    p = ctrl.PIDParams()
    p.Kp = 30.0; p.Ki = 5.0; p.Kd = 2.0; p.N = 50.0; p.Kb = 1.0
    p.uMin = -20.0; p.uMax = 20.0
    return ctrl.DiscretePID(p, TS)


# ---------------------------------------------------------------------------
# ILC P-type
# ---------------------------------------------------------------------------
pp = ctrl.ILCParams()
pp.N = N; pp.Ts = TS
pp.mode = ctrl.ILCMode.PType
pp.Lp = 0.5; pp.Q_filter = 0.95
pp.uMin = -20.0; pp.uMax = 20.0
ilc_p = ctrl.ILC(pp)

# ILC norm-optimal
pno = ctrl.ILCParams()
pno.N = N; pno.Ts = TS
pno.mode = ctrl.ILCMode.NormOptimal
pno.rho_u = 0.1; pno.rho_e = 1.0
pno.uMin = -20.0; pno.uMax = 20.0
ilc_no = ctrl.ILC(pno, G_eigen)


# ---------------------------------------------------------------------------
# Run trials
# ---------------------------------------------------------------------------
def run_trials(ilc, n_trials, label):
    pid   = make_pid()
    plant = Plant()
    rms_hist = []
    for _ in range(n_trials):
        pid.reset(); plant.reset()
        sse = 0.0
        for k in range(N):
            y = plant.step(0.0)
            e = r[k] - y
            u = ilc.feedforward(k) + float(pid.compute(e))
            plant.step(u)
            ilc.record_error(k, e)
            sse += e ** 2
        ilc.update_feedforward()
        rms_hist.append(np.sqrt(sse / N))
    print(f"\n{label}")
    print(f"  Trial 1  RMS: {rms_hist[0]:.5f}")
    print(f"  Trial {n_trials} RMS: {rms_hist[-1]:.5f}")
    print(f"  Reduction: {rms_hist[0]/rms_hist[-1]:.1f}x")
    return rms_hist


# Baseline PID (no ILC)
pid_only = make_pid(); plant_b = Plant()
rms_pid = []
for _ in range(5):
    pid_only.reset(); plant_b.reset()
    sse = 0.0
    for k in range(N):
        y = plant_b.step(0.0)
        e = r[k] - y
        plant_b.step(float(pid_only.compute(e)))
        sse += e ** 2
    rms_pid.append(np.sqrt(sse / N))

print("=== ILC Demo ===")
print(f"\nPID only (no ILC): RMS = {np.mean(rms_pid):.5f} (stable, no improvement)")

rms_p  = run_trials(ilc_p,  N_TRIALS, f"ILC P-type (Lp=0.5, Q=0.95) over {N_TRIALS} trials")
rms_no = run_trials(ilc_no, N_TRIALS, f"ILC Norm-optimal (rho_u=0.1) over {N_TRIALS} trials")


# ---------------------------------------------------------------------------
# Verification
# ---------------------------------------------------------------------------
assert rms_p[-1]  < rms_p[0]  * 0.5, f"[FAIL] P-type did not converge: {rms_p[-1]:.5f}"
assert rms_no[-1] < rms_no[0] * 0.5, f"[FAIL] Norm-opt did not converge: {rms_no[-1]:.5f}"
assert rms_no[-1] <= rms_p[-1] * 1.5, "[FAIL] Norm-optimal should not be worse than P-type"
assert ilc_p.trial_index()  == N_TRIALS, "[FAIL] P-type trial count mismatch"
assert ilc_no.trial_index() == N_TRIALS, "[FAIL] Norm-opt trial count mismatch"

print("\n[PASS] All ILC ex90 checks passed.")
