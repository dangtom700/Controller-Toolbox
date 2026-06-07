"""
drill_string_plant.py
Torsional drill string plant with Stribeck friction (stick-slip model).

States: x = [phi, omega_b]
  phi     - twist angle between top-drive and bit [rad]
  omega_b - bit angular velocity [rad/s]

Input:  omega_t  - top-drive angular velocity command [rad/s]
Output: omega_b  - bit angular velocity [rad/s]

Equations (continuous time):
  d(phi)/dt    = omega_t - omega_b
  d(omega_b)/dt = (k_t*phi + c_t*(omega_t - omega_b) - T_bit(omega_b)) / J_b

  T_bit(omega_b) = WOB*R_b * (mu_k + (mu_s-mu_k)*exp(-|omega_b|/eps_v))
                            * tanh(omega_b/eps_tanh)

Integration: 4th-order Runge-Kutta at Ts.
"""

import math


class DrillStringPlant:
    def __init__(self, params: dict):
        self.J_b      = params['J_b']
        self.k_t      = params['k_t']
        self.c_t      = params['c_t']
        self.WOB      = params['WOB']
        self.R_b      = params['R_b']
        self.mu_s     = params['mu_s']
        self.mu_k     = params['mu_k']
        self.eps_v    = params['eps_v']
        self.eps_tanh = params['eps_tanh']
        self.Ts       = params['Ts']
        self.phi      = 0.0
        self.omega_b  = 0.0

    def T_bit(self, omega_b: float) -> float:
        """Stribeck friction torque at the bit [N*m]."""
        T0       = self.WOB * self.R_b
        stribeck = self.mu_k + (self.mu_s - self.mu_k) * math.exp(-abs(omega_b) / self.eps_v)
        return T0 * stribeck * math.tanh(omega_b / self.eps_tanh)

    def dT_bit_domega(self, omega_b: float) -> float:
        """Analytical derivative of T_bit w.r.t. omega_b [N*m/(rad/s)]."""
        T0    = self.WOB * self.R_b
        eps_v = self.eps_v; eps_th = self.eps_tanh
        sign  = 1.0 if omega_b >= 0 else -1.0
        exp_  = math.exp(-abs(omega_b) / eps_v)
        strib = self.mu_k + (self.mu_s - self.mu_k) * exp_
        d_s   = -(self.mu_s - self.mu_k) / eps_v * exp_ * sign
        th    = math.tanh(omega_b / eps_th)
        dth   = 1.0 / (eps_th * math.cosh(omega_b / eps_th) ** 2)
        return T0 * (d_s * th + strib * dth)

    def _derivs(self, phi: float, omega_b: float, omega_t: float):
        dphi     = omega_t - omega_b
        domega_b = (self.k_t * phi + self.c_t * (omega_t - omega_b)
                    - self.T_bit(omega_b)) / self.J_b
        return dphi, domega_b

    def step(self, omega_t: float) -> float:
        """Advance one time step (RK4). Returns new omega_b."""
        dt = self.Ts
        p, ob = self.phi, self.omega_b

        k1p, k1o = self._derivs(p,              ob,              omega_t)
        k2p, k2o = self._derivs(p + dt/2*k1p,   ob + dt/2*k1o,  omega_t)
        k3p, k3o = self._derivs(p + dt/2*k2p,   ob + dt/2*k2o,  omega_t)
        k4p, k4o = self._derivs(p + dt*k3p,      ob + dt*k3o,    omega_t)

        self.phi     += dt / 6.0 * (k1p + 2*k2p + 2*k3p + k4p)
        self.omega_b += dt / 6.0 * (k1o + 2*k2o + 2*k3o + k4o)
        return self.omega_b

    def reset(self, phi: float = 0.0, omega_b: float = 0.0):
        self.phi     = phi
        self.omega_b = omega_b

    def state(self):
        return (self.phi, self.omega_b)

    def phi_ss(self, omega_ref: float) -> float:
        """Equilibrium twist angle for steady bit speed omega_ref."""
        if abs(omega_ref) < 1e-6:
            return 0.0
        return self.T_bit(omega_ref) / self.k_t
