import numpy as np

# Tug geometry from plant_params.json
stations = [
    (60.0,  25.0, np.radians(-120.0)),
    (-60.0,  25.0, np.radians(-60.0)),
    (-60.0, -25.0, np.radians(60.0)),
    (60.0, -25.0, np.radians(120.0)),
]

B = np.zeros((3, 4))
for i, (x, y, a) in enumerate(stations):
    ca, sa = np.cos(a), np.sin(a)
    B[0, i] = ca
    B[1, i] = sa
    B[2, i] = -y * ca + x * sa

print("B matrix:")
print(np.round(B, 4))

BBT = B @ B.T
print("\nBB^T:")
print(np.round(BBT, 2))

B_pinv = B.T @ np.linalg.inv(BBT)
print("\nB_pinv (4x3):")
print(np.round(B_pinv, 6))

tau_c = np.array([0.0, -775000.0, 0.0])
T_unc = B_pinv @ tau_c
print(f"\nFor tau_c = {tau_c}:")
print(f"T_unc = {T_unc}")
print(f"Achieved = {B @ np.maximum(T_unc, 0)}")
print(f"Achieved with T3,T4 clamped to 0 (T={np.maximum(T_unc,0)}): tau_y={B[1,:] @ np.maximum(T_unc,0):.0f}")

# Check what happens with tau_y = -2e6 (TAU_XY_MAX)
tau_max = np.array([0.0, -2e6, 0.0])
T_max_unc = B_pinv @ tau_max
print(f"\nFor tau_c = {tau_max} (TAU_XY_MAX):")
print(f"T_unc = {T_max_unc}")
