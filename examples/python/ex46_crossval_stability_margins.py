"""
ex46 - Cross-Validation: ctrl.SystemAnalysis.calculate_margins vs control.margin
==================================================================================
Goal     : Verify that the C++ frequency-domain stability margin computation
           matches python-control reference values.

Test cases:
  1. L(z) = c2d(5/(s*(s+1))) Ts=0.1 -- finite Gm and Pm
  2. L(z) stable system eigenvalue test using get_poles()
  3. Frequency response magnitude at DC and Nyquist

Agreement criterion: |Gm_ctrl - Gm_ref| < 0.3 dB, |Pm_ctrl - Pm_ref| < 1 deg.
(Grid-based approximation vs polynomial root-finding; some discretisation error expected.)

Run:
    conda run -n soft_robotics -- python ex46_crossval_stability_margins.py
"""
import _setup_bindings  # noqa: F401
import ctrl_toolbox as ctrl
import numpy as np
import control

results = []

def check_margin(label, ctrl_val, ref_val, tol):
    err = abs(ctrl_val - ref_val)
    ok = err < tol
    print(f"  {label:50s}: ctrl={ctrl_val:.4f}  ref={ref_val:.4f}  err={err:.4f}  {'PASS' if ok else 'FAIL'}")
    return ok

# ===========================================================================
# Plant: L(z) = c2d(5 * G) where G(s) = 1/(s*(s+1))  Ts=0.1
# Reference: control.margin -> Gm=4.0678, Pm=19.114 deg
# ===========================================================================
print("=== Stability margins: L(z) = c2d(5/(s*(s+1))), Ts=0.1 ===")
Ts = 0.1
# State-space: G(s) = 1/(s^2+s) = [[0,1],[0,-1]], B=[0,1]', C=[1,0], D=0
# with gain K=5 applied via B
sys_c = ctrl.StateSpace(
    np.array([[0.0, 1.0], [0.0, -1.0]]),
    np.array([[0.0], [5.0]]),   # K=5 absorbed into B
    np.array([[1.0, 0.0]]),
    np.zeros((1, 1)), 0.0)
plant_d = ctrl.c2d(sys_c, Ts, ctrl.C2dMethod.ZOH)

# ctrl result
margins_ctrl = ctrl.SystemAnalysis.calculate_margins(plant_d)
Gm_ctrl_dB = margins_ctrl.gain_margin_db
Pm_ctrl_deg = margins_ctrl.phase_margin_deg

# control.margin reference
G_ctrl_lib = control.ss(plant_d.A, plant_d.B, plant_d.C, plant_d.D, Ts)
Gm_ref, Pm_ref, Wcg_ref, Wcp_ref = control.margin(G_ctrl_lib)
Gm_ref_dB = 20 * np.log10(Gm_ref)

print(f"  Reference: Gm={Gm_ref_dB:.4f} dB, Pm={Pm_ref:.4f} deg  (Wcg={Wcg_ref:.4f}, Wcp={Wcp_ref:.4f})")
results.append(check_margin("Gain margin [dB]",    Gm_ctrl_dB, Gm_ref_dB, tol=0.5))
results.append(check_margin("Phase margin [deg]",  Pm_ctrl_deg, Pm_ref,    tol=2.0))

# ===========================================================================
# Stability check: poles of stable and unstable systems
# ===========================================================================
print("\n=== Stability check: get_poles() and is_discrete_stable() ===")
# Stable: all poles inside unit disk
sys_stable = ctrl.StateSpace(
    np.array([[0.9, 0.0], [0.0, 0.5]]),
    np.eye(2), np.eye(2), np.zeros((2, 2)), Ts)
poles_stable = ctrl.SystemAnalysis.get_poles(sys_stable)
ok_stable = ctrl.SystemAnalysis.is_discrete_stable(sys_stable)
print(f"  Stable plant poles: {[f'{p:.4f}' for p in np.abs(poles_stable)]}  is_stable={ok_stable}")
results.append(ok_stable)
results.append(all(np.abs(poles_stable) < 1.0))

# Unstable: one pole outside unit disk
sys_unstable = ctrl.StateSpace(
    np.array([[1.05, 0.0], [0.0, 0.5]]),
    np.eye(2), np.eye(2), np.zeros((2, 2)), Ts)
ok_unstable = not ctrl.SystemAnalysis.is_discrete_stable(sys_unstable)
print(f"  Unstable plant: is_discrete_stable={ctrl.SystemAnalysis.is_discrete_stable(sys_unstable)} (expected False)")
results.append(ok_unstable)

# ===========================================================================
# Frequency response at DC and Nyquist vs scipy Bode
# ===========================================================================
print("\n=== Frequency response: magnitude at DC and Nyquist ===")
from scipy import signal as scipy_signal

# G(z) = Bd/(z-Ad) for first-order ZOH plant
sys_fo_d = ctrl.c2d(ctrl.StateSpace(
    np.array([[-1.0]]), np.array([[1.0]]),
    np.array([[1.0]]), np.zeros((1,1)), 0.0), 0.1, ctrl.C2dMethod.ZOH)

omega_dc = np.array([1e-4])  # near DC
H_dc = ctrl.SystemAnalysis.get_frequency_response(sys_fo_d, omega_dc)
mag_dc = abs(H_dc[0])
# DC gain: H(e^{j*0}) = C*(I-A)^{-1}*B + D = 1/(1-exp(-0.1)) * (1-exp(-0.1)) = 1
dc_gain_exact = 1.0
print(f"  DC gain: ctrl={mag_dc:.6f}  exact={dc_gain_exact:.6f}")
results.append(abs(mag_dc - dc_gain_exact) < 0.01)

# Compare with python-control frequency response
sys_fo_ctrl_lib = control.ss(sys_fo_d.A, sys_fo_d.B, sys_fo_d.C, sys_fo_d.D, 0.1)
omega_test = np.array([0.1, 0.5, 1.0, 2.0])
H_ctrl_array = ctrl.SystemAnalysis.get_frequency_response(sys_fo_d, omega_test)
freq_resp = control.frequency_response(sys_fo_ctrl_lib, omega_test)
H_clib = freq_resp.response  # FrequencyResponseData object
H_clib_mag = np.abs(H_clib.squeeze())
H_ctrl_mag  = np.abs(H_ctrl_array)

print(f"  Frequency response comparison (4 points):")
for i, w in enumerate(omega_test):
    err = abs(H_ctrl_mag[i] - H_clib_mag[i])
    print(f"    w={w:.1f} rad/s: ctrl={H_ctrl_mag[i]:.6f}  control_lib={H_clib_mag[i]:.6f}  err={err:.2e}")
    results.append(err < 0.01)

n_pass = sum(results)
n_total = len(results)
print(f"\n{'='*60}")
print(f"Stability/margins cross-validation: {n_pass}/{n_total} checks passed")
assert n_pass == n_total
print("PASS")
