"""
ex37 - MPC vs LQR: DC Motor Position Control (ctrl_toolbox bindings)
=====================================================================
Goal     : Side-by-side comparison of DiscreteMPC and DiscreteLQR on a
           DC motor position control task. Both use the same plant and
           roughly equivalent aggressiveness.

Scenario : Armature-controlled DC motor (J=0.01, b=0.1, Km=0.01, Ra=1 Ohm).
           ZOH @ Ts=0.05 s. Reference = 1.0 rad step. Voltage limited to +-12V.

Run:
    conda run -n soft_robotics -- python ex37_mpc_lqr_dc_motor.py
"""
import _setup_bindings  # noqa: F401
import ctrl_toolbox as ctrl
import numpy as np

J = 0.01; b = 0.1; Km = 0.01; Ra = 1.0; Ts = 0.05
plant = ctrl.c2d(ctrl.StateSpace(
    np.array([[0.0, 1.0], [0.0, -b/J]]),
    np.array([[0.0], [Km/(J*Ra)]]),
    np.array([[1.0, 0.0]]), np.zeros((1,1)), 0.0), Ts, ctrl.C2dMethod.ZOH)

lqr_p = ctrl.LQRParams()
lqr_p.Q = np.diag([100.0, 1.0]); lqr_p.R = np.array([[0.01]])
lqr = ctrl.DiscreteLQR(plant, lqr_p)

mp = ctrl.MPCParams()
mp.Np=30; mp.Nc=8; mp.rho_y=100.0; mp.rho_u=0.01
mp.uMin=-12.0; mp.uMax=12.0; mp.duMin=-5.0; mp.duMax=5.0
mp.qp_max_iter = 1000  # default 200 too low for Np=30 with tight rho_u=0.01
mpc = ctrl.DiscreteMPC(plant, mp)

N = int(3.0/Ts); r_pos = 1.0
x_lqr = np.zeros(2); x_mpc = np.zeros(2)
x_ref_lqr = np.array([r_pos, 0.0]); r_ref = np.array([r_pos])
pos_lqr, pos_mpc = [], []

for k in range(N):
    u_lqr = float(np.clip(lqr.compute(x_lqr, x_ref_lqr)[0], mp.uMin, mp.uMax))
    _, x_lqr = ctrl.ss_step_copy(plant, x_lqr, np.array([u_lqr]))
    pos_lqr.append(x_lqr[0])

    mpc.set_state(x_mpc)
    u_mpc = float(mpc.compute_ref(x_mpc, r_ref)[0])
    _, x_mpc = ctrl.ss_step_copy(plant, x_mpc, np.array([u_mpc]))
    pos_mpc.append(x_mpc[0])

pos_lqr = np.array(pos_lqr); pos_mpc = np.array(pos_mpc)

print("=== DC Motor Position: MPC vs LQR ===")
print(f"  LQR final: {pos_lqr[-1]:.4f} rad  IAE={np.sum(np.abs(pos_lqr-r_pos))*Ts:.4f} rad.s")
print(f"  MPC final: {pos_mpc[-1]:.4f} rad  IAE={np.sum(np.abs(pos_mpc-r_pos))*Ts:.4f} rad.s")
assert abs(pos_lqr[-1] - r_pos) < 0.02, "LQR did not settle"
assert abs(pos_mpc[-1] - r_pos) < 0.02, "MPC did not settle"
print("PASS")
