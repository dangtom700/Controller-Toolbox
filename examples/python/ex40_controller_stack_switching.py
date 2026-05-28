"""
ex40 - ControllerStack Supervisory Switching (ctrl_toolbox bindings)
====================================================================
Goal     : Demonstrate ControllerStack in Supervisory mode with explicit
           enable/disable switching: MPC -> GPC -> PID fallback chain.

Scenario : Reactor level control. Three switching events:
           t= 0s: MPC active (normal)
           t=50s: MPC disabled (fault simulation) -> GPC takes over
           t=100s: GPC disabled -> PID (always-eligible fallback)
           t=150s: MPC/GPC restored -> MPC resumes

Run:
    conda run -n soft_robotics -- python ex40_controller_stack_switching.py
"""
import _setup_bindings  # noqa: F401
import ctrl_toolbox as ctrl
import numpy as np

Ts=0.1; r_sp=1.0
a=np.exp(-Ts/5.0)
plant=ctrl.StateSpace(np.array([[a]]),np.array([[(1-a)]]),
                      np.array([[1.0]]),np.zeros((1,1)),Ts)

mp=ctrl.MPCParams(); mp.Np=20; mp.Nc=5; mp.rho_y=10.0; mp.rho_u=0.1
mp.uMin=-5.0; mp.uMax=5.0
mpc=ctrl.DiscreteMPC(plant,mp)

gp=ctrl.GPCParams(); gp.Np=15; gp.Nu=4; gp.rho_y=10.0; gp.rho_u=0.1
gp.uMin=-5.0; gp.uMax=5.0
gpc=ctrl.GeneralizedPredictiveController(plant,gp)

pp=ctrl.PIDParams(); pp.Kp=2.0; pp.Ki=0.5; pp.N=20.0; pp.uMin=-5.0; pp.uMax=5.0
pid=ctrl.DiscretePID(pp,Ts)

stack=ctrl.ControllerStack(ctrl.StackMode.Supervisory, Ts)
stack.add_controller(mpc,"MPC")
stack.add_controller(gpc,"GPC")
stack.add_controller(pid,"PID")

N=int(200.0/Ts); x=np.zeros(1); log=[]

for k in range(N):
    t=k*Ts
    if 50<=t<100:
        stack.set_active("MPC",False); stack.set_active("GPC",True)
    elif 100<=t<150:
        stack.set_active("MPC",False); stack.set_active("GPC",False)
    else:
        stack.set_active("MPC",True); stack.set_active("GPC",True)

    y=float(x[0]); e=r_sp-y
    u=float(np.clip(stack.compute(e), mp.uMin, mp.uMax))
    _,x=ctrl.ss_step_copy(plant, x, np.array([u]))
    log.append([t, y, stack.active_controller_name()])

names={n for t,y,n in log if t<49}
fback={n for t,y,n in log if 51<=t<=99}
pid_f={n for t,y,n in log if 101<=t<=149}
rest ={n for t,y,n in log if t>=151}
final=log[-1][1]

print("=== ControllerStack Supervisory Switching ===")
print(f"t<50  (normal): {names}")
print(f"t=50..100 (MPC disabled): {fback}")
print(f"t=100..150 (MPC+GPC disabled): {pid_f}")
print(f"t>150 (restored): {rest}")
print(f"Final level: {final:.4f} m")
assert names=={"MPC"}; assert fback=={"GPC"}; assert pid_f=={"PID"}; assert rest=={"MPC"}
assert abs(final-r_sp)<0.02
print("PASS")
