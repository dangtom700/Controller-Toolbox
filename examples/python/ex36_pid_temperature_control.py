"""
ex36 - PID Temperature Control (ctrl_toolbox bindings)
=======================================================
Goal     : Demonstrate DiscretePID with derivative-on-measurement (DoM) for
           industrial heater temperature regulation. Bindings example for the
           ctrl_toolbox C++ library.

Scenario : First-order thermal plant G(s) = 2/(30s+1), ZOH @ Ts=1s.
           Setpoint step from 20 C to 80 C. DoM prevents derivative kick.

Run:
    conda run -n soft_robotics -- python ex36_pid_temperature_control.py
"""
import _setup_bindings  # noqa: F401
import ctrl_toolbox as ctrl
import numpy as np

Ts = 1.0; K_p = 2.0; tau_p = 30.0

a = np.exp(-Ts / tau_p)
plant = ctrl.StateSpace(np.array([[a]]), np.array([[K_p*(1-a)]]),
                        np.array([[1.0]]), np.zeros((1,1)), Ts)

lam = tau_p / 3.0
p = ctrl.PIDParams()
p.Kp = tau_p / (K_p * lam); p.Ki = p.Kp / tau_p; p.Kd = 0.0; p.N = 10.0
p.uMin = 0.0; p.uMax = 100.0
pid = ctrl.DiscretePID(p, Ts)

N = 300; r = 80.0; T_ambient = 20.0
x = np.array([T_ambient]); u = 0.0
y_arr, e_arr = [], []

for k in range(N):
    y, x = ctrl.ss_step_copy(plant, x, np.array([u]))
    y_val = float(y[0])
    u = pid.compute_dom(y_val, r)
    u = float(np.clip(u, p.uMin, p.uMax))
    y_arr.append(y_val); e_arr.append(r - y_val)

y_arr = np.array(y_arr)
IAE = np.sum(np.abs(e_arr)) * Ts

print("=== PID Temperature Control ===")
print(f"Setpoint:            {r:.1f} C")
print(f"Steady-state output: {y_arr[-1]:.2f} C")
print(f"IAE:                 {IAE:.1f} C.s")
assert abs(y_arr[-1] - r) < 0.1, f"Steady-state error too large: {y_arr[-1]:.2f} C"
print("PASS")
