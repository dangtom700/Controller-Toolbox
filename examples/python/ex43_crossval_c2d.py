"""
ex43 - Cross-Validation: ctrl.c2d vs scipy.signal.cont2discrete / control.c2d
==============================================================================
Goal     : Verify that the C++ c2d() implementation (ZOH and Tustin) matches
           scipy and python-control reference values to machine precision.

Plants tested:
  1. G(s) = 1/(s^2 + 1.5s + 1)  ZOH @ Ts=0.01
  2. G(s) = 1/(s+1)              ZOH @ Ts=0.1
  3. G(s) = 1/(s+1)              Tustin @ Ts=0.1

Agreement criterion: |ctrl - scipy| < 1e-8 for all A, B matrix entries.
This catches sign errors, scale errors, and algorithm deviations.

Run:
    conda run -n soft_robotics -- python ex43_crossval_c2d.py
"""
import _setup_bindings  # noqa: F401
import ctrl_toolbox as ctrl
import numpy as np
from scipy import signal

TOL = 1e-8

def check(label, ctrl_val, ref_val, tol=TOL):
    err = abs(ctrl_val - ref_val)
    ok = err < tol
    marker = "PASS" if ok else f"FAIL (err={err:.2e})"
    print(f"  {label:40s}: {marker}")
    return ok

results = []

# ===========================================================================
# Plant 1: G(s) = 1/(s^2 + 1.5s + 1), ZOH @ Ts=0.01
# Controllable canonical continuous state-space:
#   Ac = [[0, 1], [-1, -1.5]],  Bc = [[0], [1]]
# ===========================================================================
print("=== c2d ZOH: G(s)=1/(s^2+1.5s+1), Ts=0.01 ===")
Ac2 = np.array([[0.0, 1.0], [-1.0, -1.5]])
Bc2 = np.array([[0.0], [1.0]])
Cc2 = np.eye(2)
Dc2 = np.zeros((2, 1))
Ts01 = 0.01

# scipy reference
Ad_ref, Bd_ref, _, _, _ = signal.cont2discrete((Ac2, Bc2, Cc2, Dc2), Ts01, method='zoh')

# ctrl reference
sys_c = ctrl.StateSpace(Ac2, Bc2, Cc2, Dc2, 0.0)
sys_d = ctrl.c2d(sys_c, Ts01, ctrl.C2dMethod.ZOH)
Ad_ctrl = sys_d.A
Bd_ctrl = sys_d.B

for i in range(2):
    for j in range(2):
        results.append(check(f"Ad[{i},{j}]", Ad_ctrl[i,j], Ad_ref[i,j]))
for i in range(2):
    results.append(check(f"Bd[{i}]", Bd_ctrl[i,0], Bd_ref[i,0]))

# ===========================================================================
# Plant 2: G(s) = 1/(s+1), ZOH @ Ts=0.1
# Exact: Ad = exp(-0.1), Bd = 1 - exp(-0.1)
# ===========================================================================
print("\n=== c2d ZOH: G(s)=1/(s+1), Ts=0.1 ===")
Ac1 = np.array([[-1.0]])
Bc1 = np.array([[1.0]])
Ts1 = 0.1

Ad_ref_z1, Bd_ref_z1, _, _, _ = signal.cont2discrete(
    (Ac1, Bc1, np.array([[1.0]]), np.array([[0.0]])), Ts1, method='zoh')
sys_c1 = ctrl.StateSpace(Ac1, Bc1, np.array([[1.0]]), np.zeros((1,1)), 0.0)
sys_d1 = ctrl.c2d(sys_c1, Ts1, ctrl.C2dMethod.ZOH)

Ad_exact = np.exp(-Ts1)
Bd_exact = 1.0 - np.exp(-Ts1)
results.append(check("Ad vs scipy",    sys_d1.A[0,0], Ad_ref_z1[0,0]))
results.append(check("Bd vs scipy",    sys_d1.B[0,0], Bd_ref_z1[0,0]))
results.append(check("Ad vs exact",    sys_d1.A[0,0], Ad_exact, tol=1e-10))
results.append(check("Bd vs exact",    sys_d1.B[0,0], Bd_exact, tol=1e-10))

# ===========================================================================
# Plant 3: G(s) = 1/(s+1), Tustin @ Ts=0.1
# Exact bilinear: Ad = (1 - Ts/2)/(1 + Ts/2) = (1-0.05)/(1+0.05) = 19/21
#                 Bd = Ts/(1 + Ts/2) = 0.1/1.05 = 2/21
# ===========================================================================
print("\n=== c2d Tustin: G(s)=1/(s+1), Ts=0.1 ===")
Ad_ref_t1, Bd_ref_t1, _, _, _ = signal.cont2discrete(
    (Ac1, Bc1, np.array([[1.0]]), np.array([[0.0]])), Ts1, method='bilinear')
sys_d1_t = ctrl.c2d(sys_c1, Ts1, ctrl.C2dMethod.Tustin)

Ad_tustin_exact = (1.0 - Ts1/2.0) / (1.0 + Ts1/2.0)  # = 0.9047619...
Bd_tustin_exact = Ts1 / (1.0 + Ts1/2.0)                # = 0.0952380...
results.append(check("Ad vs scipy",    sys_d1_t.A[0,0], Ad_ref_t1[0,0]))
results.append(check("Bd vs scipy",    sys_d1_t.B[0,0], Bd_ref_t1[0,0]))
results.append(check("Ad vs exact",    sys_d1_t.A[0,0], Ad_tustin_exact, tol=1e-10))
results.append(check("Bd vs exact",    sys_d1_t.B[0,0], Bd_tustin_exact, tol=1e-10))
# ZOH and Tustin give DIFFERENT results (not interchangeable)
print(f"  ZOH Ad={sys_d1.A[0,0]:.10f}  Tustin Ad={sys_d1_t.A[0,0]:.10f}  (differ: {abs(sys_d1.A[0,0]-sys_d1_t.A[0,0]):.2e})")

# Eigenvalue preservation: discrete poles must equal continuous poles mapped by method
cl_poles_ctrl = ctrl.SystemAnalysis.get_poles(sys_d1)
print(f"  ZOH discrete pole: {cl_poles_ctrl[0].real:.10f} (= exp(-0.1) = {np.exp(-0.1):.10f})")
results.append(check("ZOH pole vs exp(-0.1)", cl_poles_ctrl[0].real, np.exp(-Ts1), tol=1e-10))

n_pass = sum(results)
n_total = len(results)
print(f"\n{'='*50}")
print(f"c2d cross-validation: {n_pass}/{n_total} checks passed")
assert n_pass == n_total, f"{n_total-n_pass} checks failed"
print("PASS")
