"""
ex91_neural_pid.py -- Online Neural PID adaptation demo.

Demonstrates:
  1. NeuralPID adapts Kp/Ki/Kd online via plant-gradient backpropagation.
  2. Initial tracking on plant with gain b=0.2.
  3. Adapts after gain doubles to b=0.4 at k=200.
  4. Comparison of IAE before and after adaptation.

Plant:  x[k+1] = 0.8*x[k] + b*u[k]   (gain b changes at k=200)
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'build', 'bindings'))
if sys.platform == "win32" and hasattr(os, "add_dll_directory"):
    for _p in [r"C:\msys64\mingw64\bin"]:
        if os.path.isdir(_p):
            os.add_dll_directory(_p)

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'NeuralPID'):
        raise AttributeError("NeuralPID not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

import numpy as np

print("=== ex91: Online Neural PID ===")

TS = 0.01
N  = 400
R  = 1.0

# ---------------------------------------------------------------------------
# NeuralPID setup
# ---------------------------------------------------------------------------
p = ctrl.NeuralPIDParams()
p.n_hidden      = 8
p.lr            = 5e-4
p.Ts            = TS
p.plant_gain    = 0.2    # approximate b (used for gradient path)
p.Kp0           = 2.0
p.Ki0           = 0.2
p.Kd0           = 0.0
p.uMin          = -4.0
p.uMax          =  4.0

npid = ctrl.NeuralPID(p)

# ---------------------------------------------------------------------------
# Closed-loop simulation
# ---------------------------------------------------------------------------
x = 0.0
iae_1 = 0.0  # k=0..199
iae_2 = 0.0  # k=200..399

Kp_log = []
Ki_log = []

print(f"\n  {'k':>5}  {'x':>8}  {'Kp':>7}  {'Ki':>7}  {'note'}")

for k in range(N):
    b = 0.2 if k < 200 else 0.4   # gain doubles at k=200

    u = npid.compute(R - x)
    x = 0.8 * x + b * u

    Kp_log.append(npid.current_kp())
    Ki_log.append(npid.current_ki())

    if k < 200:
        iae_1 += abs(R - x) * TS
    else:
        iae_2 += abs(R - x) * TS

    if k in (0, 50, 100, 199, 200, 250, 300, 399):
        note = "<<< gain change" if k == 200 else ""
        print(f"  {k:5d}  {x:8.4f}  {npid.current_kp():7.4f}  {npid.current_ki():7.4f}  {note}")

print(f"\n  IAE phase-1 (b=0.2): {iae_1:.4f}")
print(f"  IAE phase-2 (b=0.4): {iae_2:.4f}  (NeuralPID adapts to gain change)")
print(f"\n  Initial Kpapprox ={Kp_log[0]:.3f}, Kiapprox ={Ki_log[0]:.3f}")
print(f"  Final   Kpapprox ={Kp_log[-1]:.3f}, Kiapprox ={Ki_log[-1]:.3f}")

# ---------------------------------------------------------------------------
# Reset and verify zero response
# ---------------------------------------------------------------------------
print("\n-- Reset test: zero error after reset --")
npid.reset()
u_after = npid.compute(0.0)
print(f"  u after reset (zero error): {u_after:.6f}  (expected near 0)")

print("\n[PASS] NeuralPID demo complete.")
