"""
ex42 - IControllerObserver Telemetry (ctrl_toolbox bindings)
=============================================================
Goal     : Demonstrate the Observer pattern for non-intrusive controller
           telemetry logging. No modification of controller code is needed.

Scenario : PID heater control. A Python subclass of IControllerObserver
           receives every compute() callback, logging (t, u, error).
           Post-run analysis detects actuator saturation events.

Run:
    conda run -n soft_robotics -- python ex42_observer_telemetry.py
"""
import _setup_bindings  # noqa: F401
import ctrl_toolbox as ctrl
import numpy as np

Ts=1.0; a=np.exp(-Ts/30.0); K=2.0
plant=ctrl.StateSpace(np.array([[a]]),np.array([[K*(1-a)]]),
                      np.array([[1.0]]),np.zeros((1,1)),Ts)

pp=ctrl.PIDParams(); pp.Kp=2.0; pp.Ki=0.1; pp.N=5.0; pp.uMin=0.0; pp.uMax=100.0
pid=ctrl.DiscretePID(pp, Ts)

class Telemetry(ctrl.IControllerObserver):
    def __init__(self):
        super().__init__()
        self._k=0; self.u=[]; self.err=[]
    def on_compute(self, u, signal):
        self.u.append(u); self.err.append(signal); self._k+=1

tel=Telemetry()
pid.attach_observer(tel)

N=200; r=80.0; x=np.array([20.0])

for k in range(N):
    y,x=ctrl.ss_step_copy(plant, x, np.array([tel.u[-1] if tel.u else 0.0]))
    pid.compute(r-float(y[0]))   # observer fires inside here

u_arr=np.array(tel.u)
n_sat=int((u_arr>=pp.uMax-0.1).sum())

print("=== IControllerObserver Telemetry ===")
print(f"Callbacks received: {len(tel.u)}")
print(f"Saturation events:  {n_sat} ({100*n_sat/N:.1f}%)")
print(f"Final temperature:  {float(y[0]):.2f} C  (target {r})")
assert len(tel.u)==N, "Observer call count mismatch"
assert n_sat>0, "Should saturate during warm-up"
assert abs(float(y[0])-r)<1.0, "Temperature did not settle"
print("PASS")
