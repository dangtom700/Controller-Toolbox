"""
ex47 - Cross-Validation: KalmanFilter steady-state vs scipy DARE dual solution
===============================================================================
Goal     : Verify that KalmanFilter covariance P[k|k] (updated) converges to the
           steady-state value derived from the dual DARE, and that the implied
           Kalman gain matches scipy / python-control reference values.

KEY CONVENTION NOTE
-------------------
scipy.linalg.solve_discrete_are(A.T, C.T, Q, R) returns P_pred (the PREDICTION
error covariance P[k+1|k]).  KalmanFilter.covariance() returns P_updated
(the UPDATE covariance P[k|k]).  These differ by one innovation update step:

    P_updated = (I - K*C) P_pred (I - K*C)' + K*R*K'   (Joseph form)
    K         = P_pred * C' * (C*P_pred*C' + R)^{-1}

python-control dlqe returns the predictor-form gain L = A * K_update, not K directly.

Run:
    conda run -n soft_robotics -- python ex47_crossval_kalman.py
"""
import _setup_bindings  # noqa: F401
import ctrl_toolbox as ctrl
import numpy as np
from scipy import linalg
import control

results = []

def check(label, ctrl_val, ref_val, tol=1e-4):
    err = np.max(np.abs(np.array(ctrl_val) - np.array(ref_val)))
    ok = err < tol
    print(f"  {label:55s}: {'PASS' if ok else f'FAIL (err={err:.2e})'}")
    return ok

def dare_to_kalman(A, C, Q, R):
    """Compute steady-state Kalman covariances from DARE.
    Returns: (P_pred, K_update, P_updated)
    """
    P_pred = linalg.solve_discrete_are(A.T, C.T, Q, R)
    S = C @ P_pred @ C.T + R
    K = P_pred @ C.T @ np.linalg.inv(S)
    I_n = np.eye(A.shape[0])
    IKC = I_n - K @ C
    P_upd = IKC @ P_pred @ IKC.T + K @ R @ K.T   # Joseph form
    return P_pred, K, P_upd

# ===========================================================================
# System 1: first-order, x[k+1] = 0.9*x + u,  y = x,  Q=1e-3, R=1e-2
# ===========================================================================
print("=== Kalman SS: x[k+1]=0.9x+u, y=x, Q=1e-3, R=1e-2 ===")
A1 = np.array([[0.9]]); B1 = np.array([[1.0]])
C1 = np.array([[1.0]]); D1 = np.zeros((1,1))
Q1 = np.array([[1e-3]]); R1 = np.array([[1e-2]])
Ts = 0.01

plant1 = ctrl.StateSpace(A1, B1, C1, D1, Ts)
kf1    = ctrl.KalmanFilter(plant1, Q1, R1)

P_pred1, K_upd1, P_upd1 = dare_to_kalman(A1, C1, Q1, R1)
print(f"  scipy P_pred = {P_pred1[0,0]:.8f}")
print(f"  scipy K_upd  = {K_upd1[0,0]:.8f}")
print(f"  scipy P_upd  = {P_upd1[0,0]:.8f}")

# Run KF 2000 steps with constant measurements
u1 = np.array([0.1]); y1 = np.array([0.5])
for _ in range(2000):
    kf1.step(y1, u1)

P_ctrl1 = kf1.covariance()
print(f"  ctrl P[k=2000]={P_ctrl1[0,0]:.8f}")
# kf.covariance() = P_updated (post-innovation)
results.append(check("P[k|k] vs scipy P_updated",  P_ctrl1[0,0], P_upd1[0,0], tol=1e-5))

# Verify the steady-state prediction covariance (one-step ahead) matches scipy P_pred.
# After the update step: P_pred_next = A * P_updated * A' + Q
P_pred_next = A1 @ P_ctrl1 @ A1.T + Q1
results.append(check("P[k+1|k] back from P_updated vs scipy P_pred", P_pred_next[0,0], P_pred1[0,0], tol=1e-5))

# ===========================================================================
# System 2: 2-state, partial measurement
# ===========================================================================
print("\n=== Kalman SS: 2-state, A=[[0.9,0.1],[0,0.8]], C=[1,0] ===")
A2 = np.array([[0.9, 0.1], [0.0, 0.8]])
B2 = np.array([[0.0], [0.1]])
C2 = np.array([[1.0, 0.0]])
D2 = np.zeros((1, 1))
Q2 = 0.01 * np.eye(2); R2 = np.array([[0.1]])
Ts2 = 0.01

plant2 = ctrl.StateSpace(A2, B2, C2, D2, Ts2)
kf2    = ctrl.KalmanFilter(plant2, Q2, R2)

P_pred2, K_upd2, P_upd2 = dare_to_kalman(A2, C2, Q2, R2)
print(f"  scipy P_pred:\n{P_pred2}")
print(f"  scipy K_upd:\n{K_upd2}")
print(f"  scipy P_upd:\n{P_upd2}")

u2 = np.array([0.0]); y2 = np.array([0.5])
for _ in range(3000):
    kf2.step(y2, u2)

P_ctrl2 = kf2.covariance()
print(f"  ctrl P[k=3000]:\n{P_ctrl2}")

results.append(check("P[0,0] vs scipy P_updated",  P_ctrl2[0,0], P_upd2[0,0], tol=1e-4))
results.append(check("P[1,1] vs scipy P_updated",  P_ctrl2[1,1], P_upd2[1,1], tol=1e-4))
results.append(check("P[0,1] vs scipy P_updated",  P_ctrl2[0,1], P_upd2[0,1], tol=1e-4))

# ===========================================================================
# Cross-check: control.dlqe predictor-form gain L = A * K_update
# control.dlqe implements x_hat[k+1] = A*x_hat[k] + L*(y[k] - C*x_hat[k])
# so L = A * K_update (not K_update directly)
# ===========================================================================
print("\n=== vs control.dlqe (predictor-form gain L = A*K_update) ===")
L_dlqe, P_dlqe, _ = control.dlqe(A2, np.eye(2), C2, Q2, R2)
L_expected = A2 @ K_upd2   # predictor-form: L = A * K_innovation
print(f"  control.dlqe L:\n{L_dlqe}")
print(f"  A @ K_upd:\n{L_expected}")
results.append(check("control.dlqe L[0,0] vs A*K_upd", L_dlqe[0,0], L_expected[0,0], tol=1e-6))
results.append(check("control.dlqe L[1,0] vs A*K_upd", L_dlqe[1,0], L_expected[1,0], tol=1e-6))

# ===========================================================================
# PSD check
# ===========================================================================
print("\n=== Covariance PSD check ===")
eig_min = np.linalg.eigvalsh(P_ctrl2).min()
print(f"  Min eigenvalue of P_updated: {eig_min:.2e}")
results.append(eig_min >= -1e-10)

n_pass = sum(results)
n_total = len(results)
print(f"\n{'='*60}")
print(f"Kalman cross-validation: {n_pass}/{n_total} checks passed")
assert n_pass == n_total
print("PASS")
