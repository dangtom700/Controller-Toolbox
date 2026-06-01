"""
ex39 - Adaptive GPC + RLS for Flow Control (ctrl_toolbox bindings)
===================================================================
Goal     : Demonstrate GeneralizedPredictiveController with hot-swap plant
           model from RecursiveLeastSquares - a self-tuning adaptive loop.

Scenario : Chemical flow control. Plant gain shifts from 1.0 to 1.6 at t=100s
           (valve wear). RLS re-identifies every 50 steps and updates the GPC
           predictor via set_plant(). Regulation is maintained through the shift.

Run:
    conda run -n soft_robotics -- python ex39_gpc_adaptive_rls.py
"""
import _setup_bindings  # noqa: F401
import ctrl_toolbox as ctrl
import numpy as np

rng = np.random.default_rng(0)
Ts = 0.5; r_sp = 10.0

def make_plant(K):
    a = np.exp(-Ts/10.0)
    return ctrl.StateSpace(np.array([[a]]),np.array([[K*(1-a)]]),
                           np.array([[1.0]]),np.zeros((1,1)),Ts)

plant_a = make_plant(1.0); plant_b = make_plant(1.6)

gp = ctrl.GPCParams()
gp.Np=20; gp.Nu=6; gp.rho_y=5.0; gp.rho_u=0.5; gp.alpha=0.1
gp.uMin=0.0; gp.uMax=25.0
gp.qp_max_iter = 1000  # default 200 too low for Np=20; eliminates QP convergence warnings
gpc = ctrl.GeneralizedPredictiveController(plant_a, gp)
rls = ctrl.RecursiveLeastSquares(na=1, nb=1, lambda_forgetting=0.98, Ts=Ts)

N=int(250.0/Ts); RETUNE=50
x=np.zeros(1); u=0.0; log=[]

for k in range(N):
    t=k*Ts
    pl = plant_b if t>=100.0 else plant_a
    y_true, x = ctrl.ss_step_copy(pl, x, np.array([u]))
    y = float(y_true[0]) + rng.normal(0, 0.05)
    rls.update(y, u)
    if k>20 and k%RETUNE==0:
        gpc.set_plant(rls.to_state_space())
    u = float(np.clip(gpc.compute_ref(y, r_sp), gp.uMin, gp.uMax))
    log.append([t, y])

log = np.array(log)
e1 = log[(log[:,0]>=50)&(log[:,0]<=95),1] - r_sp
e2 = log[(log[:,0]>=150)&(log[:,0]<=245),1] - r_sp

print("=== Adaptive GPC + RLS Flow Control ===")
print(f"Nominal phase: mean_err={e1.mean():.3f} L/min")
print(f"Worn plant:    mean_err={e2.mean():.3f} L/min")
assert abs(e1.mean())<0.5, "GPC failed on nominal plant"
assert abs(e2.mean())<0.5, "GPC failed after plant shift"
print("PASS")
