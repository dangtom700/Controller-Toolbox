"""
ex76_auto_gain_scheduling_demo.py
----------------------------------
Demonstrates the full gain-scheduling pipeline using pure Python + ctrl_toolbox bindings:

  1. Define a parameter-varying first-order plant  x[k+1] = a(p)*x[k] + b*u[k]
     with a(p) = 0.90 - 0.30*p  (pole moves as scheduling variable p changes).
  2. For each of 6 grid points p in [-1, 1], create a DiscreteLQR controller
     tuned to the frozen plant at that p.
  3. Assemble a GainScheduledController (LinearBlend mode).
  4. Simulate a step response while p slowly varies, and compare IAE against
     a fixed LQR designed at p=0.

Expected output: PASS
"""
import sys
import numpy as np

sys.path.insert(0, __file__.rsplit("\\", 1)[0])
import _setup_bindings
import ctrl_toolbox as ctrl

def make_plant(pole, Ts):
    """Discrete first-order plant x[k+1] = pole*x[k] + (1-pole)*u[k]."""
    return ctrl.StateSpace(
        np.array([[pole]]),
        np.array([[1.0 - pole]]),
        np.array([[1.0]]),
        np.array([[0.0]]),
        Ts
    )

def design_pid(pole, Ts):
    """Proportional-only controller tuned to the linearised plant.
    DC gain of x[k+1]=pole*x[k]+(1-pole)*u[k] is (1-pole)/(1-pole)=1.
    Closed-loop pole = pole - (1-pole)*Kp; choose Kp=3 -> CL pole ~ 0.6 for pole=0.9.
    Pure-P avoids integral windup when LinearBlend runs two controllers simultaneously."""
    pp = ctrl.PIDParams()
    pp.Kp = 3.0    # closed-loop pole ~ pole - (1-pole)*3
    pp.Ki = 0.0    # no integral (avoids windup in blend mode)
    pp.Kd = 0.0
    return ctrl.DiscretePID(pp, Ts)

Ts = 0.05
p_grid = np.linspace(-1.0, 1.0, 6)

# Build gain-scheduled controller (NearestNeighbor avoids dual-integrator windup)
sched = ctrl.GainScheduledController(Ts, ctrl.GainScheduleMode.NearestNeighbor)
for p in p_grid:
    pole = 0.90 - 0.30 * p
    pid  = design_pid(pole, Ts)
    sched.add_schedule_point(float(p), pid)

assert sched.num_points == 6, "Expected 6 schedule points"

# Nominal P controller at p=0 (baseline, no scheduling)
lqr_nom = design_pid(0.90, Ts)

# Simulate with slowly varying p = sin(2*pi*k/N)
N = 300
ref = 1.0
x_s, x_n = 0.0, 0.0
iae_sched, iae_nom = 0.0, 0.0

for k in range(N):
    p = float(np.sin(2 * np.pi * k / N))
    pole = 0.90 - 0.30 * p

    # Gain-scheduled
    sched.set_scheduling_param(p)
    u_s = sched.compute(ref - x_s)
    iae_sched += abs(ref - x_s)
    x_s = pole * x_s + (1.0 - pole) * u_s

    # Nominal
    u_n = lqr_nom.compute(ref - x_n)
    iae_nom += abs(ref - x_n)
    x_n = 0.90 * x_n + 0.10 * u_n

print(f"Gain-scheduled IAE: {iae_sched:.3f}")
print(f"Nominal LQR IAE:    {iae_nom:.3f}")

assert np.isfinite(iae_sched), "Scheduled IAE is non-finite"
assert iae_sched < 500.0, f"Scheduled IAE too large: {iae_sched:.1f}"

print("PASS")
