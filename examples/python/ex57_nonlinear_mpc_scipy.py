"""
ex57_nonlinear_mpc_scipy.py
Pure-Python Nonlinear MPC using scipy.optimize.minimize (SLSQP).

Demonstrates NMPC on a nonlinear CSTR (isothermal, concentration control).
The objective function uses scipy.integrate.odeint inside the optimizer.
No C++ changes needed - validates the plant model utilities.

Reference: pdc-master/Module_40/1_Topic/ModelPredictiveControl_1.py
"""

import numpy as np
from scipy.integrate import odeint
from scipy.optimize import minimize


# ---------------------------------------------------------------------------
# Nonlinear CSTR model (isothermal): dCa/dt = (q/V)*(Caf-Ca) - k*Ca
# ---------------------------------------------------------------------------

def cstr_ode(Ca, t, u, q=1.0, V=10.0, Caf=1.0, k=0.5):
    """CSTR: q=flow, V=volume, Caf=feed concentration, k=reaction rate."""
    q_flow = u[0]
    return (q_flow / V) * (Caf - Ca) - k * Ca


def plant_step(Ca0, u_seq, dt, steps):
    """Simulate CSTR over steps*dt with piecewise-constant inputs."""
    Ca = Ca0
    for u_k in u_seq:
        t_span = np.linspace(0, dt, 5)
        sol = odeint(cstr_ode, Ca, t_span, args=([u_k],))
        Ca = float(sol[-1, 0])
    return Ca


# ---------------------------------------------------------------------------
# NMPC
# ---------------------------------------------------------------------------

def nmpc_step(Ca_k, ref, Np, Nc, Ts, rho_y=10.0, rho_u=0.1,
              u_min=0.5, u_max=2.0, u_nom=1.0):
    """One NMPC step: returns optimal u[0] for current state Ca_k."""

    def objective(u_seq):
        Ca = Ca_k
        obj = 0.0
        u_prev = u_nom
        for i in range(Np):
            u_i = u_seq[min(i, Nc - 1)]
            # Simulate one step
            t_span = np.linspace(0, Ts, 4)
            Ca = float(odeint(cstr_ode, Ca, t_span, args=([u_i],))[-1, 0])
            obj += rho_y * (Ca - ref) ** 2 + rho_u * (u_i - u_prev) ** 2
            u_prev = u_i
        return obj

    u0 = u_nom * np.ones(Nc)
    bounds = [(u_min, u_max)] * Nc
    res = minimize(objective, u0, method='SLSQP', bounds=bounds,
                   options={'maxiter': 100, 'ftol': 1e-6})
    return res.x[0]


def main():
    Ts  = 1.0    # NMPC sample time [s]
    Np  = 8      # prediction horizon
    Nc  = 3      # control horizon
    ref = 0.25   # target concentration [mol/L] (achievable within u_max=2.0)
    N   = 40     # simulation steps

    Ca = 0.8     # initial concentration (high)
    iae, final_Ca = 0.0, 0.0

    print("Step  Ca      u")
    for k in range(N):
        u_k = nmpc_step(Ca, ref, Np, Nc, Ts)

        # Simulate one true step
        t_span = np.linspace(0, Ts, 10)
        Ca = float(odeint(cstr_ode, Ca, t_span, args=([u_k],))[-1, 0])
        iae += abs(Ca - ref) * Ts
        final_Ca = Ca

        if k % 5 == 0:
            print(f"{k:4d}  {Ca:.4f}  {u_k:.4f}")

    print(f"\nNMPC final Ca = {final_Ca:.4f}  (ref = {ref})")
    print(f"IAE = {iae:.4f}")
    assert abs(final_Ca - ref) < 0.08, f"NMPC did not converge: Ca={final_Ca:.4f}"
    print("PASS")


if __name__ == "__main__":
    main()
