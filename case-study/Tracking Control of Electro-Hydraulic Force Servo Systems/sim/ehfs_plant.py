"""
ehfs_plant.py
Nonlinear electro-hydraulic force servo plant.

States: x = [P_A (Pa), P_B (Pa), x_v (-), v_p (m/s), x_p (m)]
  P_A  - cap-side chamber pressure [Pa]
  P_B  - rod-side chamber pressure [Pa]
  x_v  - servo valve spool position (normalised -1..1)
  v_p  - piston velocity [m/s]
  x_p  - piston displacement [m]

Input:  u_v  - normalised valve command [-1, 1]
Output: F    - actuator force [N]  =  A_A*P_A - A_B*P_B

Continuous dynamics:
  dP_A/dt = beta/V_A * (Q_A - A_A*v_p - C_t*(P_A - P_B))
  dP_B/dt = beta/V_B * (Q_B + A_B*v_p + C_t*(P_A - P_B))
  dx_v/dt = (-x_v + k_v*u_v) / tau_v
  dv_p/dt = (A_A*P_A - A_B*P_B - k_L*x_p - B_v*v_p - F_fric(v_p)) / m_eff
  dx_p/dt = v_p

Valve flow (4/3 directional valve with dead-band):
  xv_eff = sign(x_v) * max(|x_v| - dead_band, 0)
  xv_pos = max(xv_eff, 0)   xv_neg = max(-xv_eff, 0)
  Q_A = Kq*(xv_pos*sqrt(max(P_S-P_A,0)) - xv_neg*sqrt(max(P_A,0)))
  Q_B = Kq*(xv_neg*sqrt(max(P_S-P_B,0)) - xv_pos*sqrt(max(P_B,0)))

Friction: F_fric = F_c * tanh(v_p / v_s)

Integration: 4th-order Runge-Kutta at Ts using N_SUBSTEPS inner steps.
"""

import math


class EHFSPlant:
    def __init__(self, params: dict):
        self.P_S       = params['P_S']
        self.A_A       = params['A_A']
        self.A_B       = params['A_B']
        self.V_A       = params['V_A']
        self.V_B       = params['V_B']
        self.beta      = params['beta']
        self.m_eff     = params['m_eff']
        self.B_v       = params['B_v']
        self.k_L       = params.get('k_L', params['k_L'])
        self.F_c       = params['F_c']
        self.v_s       = params['v_s']
        self.tau_v     = params['tau_v']
        self.Kq        = params['Kq']
        self.C_t       = params['C_t']
        self.dead_band = params['dead_band']
        self.Ts        = params['Ts']
        self.N_SUB     = int(params.get('N_SUBSTEPS', 4))

        P_A0 = params.get('P_A0', 3.0e6)
        P_B0 = params.get('P_B0', 5.0e6)
        self.P_A = P_A0
        self.P_B = P_B0
        self.x_v = 0.0
        self.v_p = 0.0
        self.x_p = 0.0

    # ------------------------------------------------------------------
    # Public accessors
    # ------------------------------------------------------------------
    def force(self) -> float:
        return self.A_A * self.P_A - self.A_B * self.P_B

    def state(self):
        return (self.P_A, self.P_B, self.x_v, self.v_p, self.x_p)

    # ------------------------------------------------------------------
    # Reset
    # ------------------------------------------------------------------
    def reset(self, P_A=None, P_B=None, x_v=0.0, v_p=0.0, x_p=0.0):
        self.P_A = P_A if P_A is not None else self._P_A0
        self.P_B = P_B if P_B is not None else self._P_B0
        self.x_v = x_v
        self.v_p = v_p
        self.x_p = x_p

    # ------------------------------------------------------------------
    # Friction model
    # ------------------------------------------------------------------
    def _F_fric(self, v: float) -> float:
        return self.F_c * math.tanh(v / self.v_s)

    # ------------------------------------------------------------------
    # Valve flow
    # ------------------------------------------------------------------
    def _valve_flows(self, x_v: float, P_A: float, P_B: float):
        xv_eff = math.copysign(max(abs(x_v) - self.dead_band, 0.0), x_v)
        xv_pos = max(xv_eff, 0.0)
        xv_neg = max(-xv_eff, 0.0)
        Q_A = self.Kq * (
            xv_pos * math.sqrt(max(self.P_S - P_A, 0.0))
            - xv_neg * math.sqrt(max(P_A, 0.0))
        )
        Q_B = self.Kq * (
            xv_neg * math.sqrt(max(self.P_S - P_B, 0.0))
            - xv_pos * math.sqrt(max(P_B, 0.0))
        )
        return Q_A, Q_B

    # ------------------------------------------------------------------
    # ODE right-hand side  (k_L can be overridden per-step for scenario s04)
    # ------------------------------------------------------------------
    def _derivs(self, P_A, P_B, x_v, v_p, x_p, u_v):
        Q_A, Q_B = self._valve_flows(x_v, P_A, P_B)

        dP_A = self.beta / self.V_A * (Q_A - self.A_A * v_p - self.C_t * (P_A - P_B))
        dP_B = self.beta / self.V_B * (Q_B + self.A_B * v_p + self.C_t * (P_A - P_B))
        dx_v = (-x_v + u_v) / self.tau_v
        dv_p = (self.A_A * P_A - self.A_B * P_B
                - self.k_L * x_p
                - self.B_v * v_p
                - self._F_fric(v_p)) / self.m_eff
        dx_p = v_p
        return dP_A, dP_B, dx_v, dv_p, dx_p

    # ------------------------------------------------------------------
    # RK4 integration step (clamps u_v and pressures to physical bounds)
    # ------------------------------------------------------------------
    def step(self, u_v: float) -> float:
        """Advance one Ts using N_SUBSTEPS RK4 inner steps. Returns F."""
        u_v = max(-1.0, min(1.0, u_v))
        dt  = self.Ts / self.N_SUB

        PA, PB, xv, vp, xp = self.P_A, self.P_B, self.x_v, self.v_p, self.x_p

        for _ in range(self.N_SUB):
            k1 = self._derivs(PA,            PB,            xv,            vp,            xp,            u_v)
            k2 = self._derivs(PA+dt/2*k1[0], PB+dt/2*k1[1], xv+dt/2*k1[2], vp+dt/2*k1[3], xp+dt/2*k1[4], u_v)
            k3 = self._derivs(PA+dt/2*k2[0], PB+dt/2*k2[1], xv+dt/2*k2[2], vp+dt/2*k2[3], xp+dt/2*k2[4], u_v)
            k4 = self._derivs(PA+dt*k3[0],   PB+dt*k3[1],   xv+dt*k3[2],   vp+dt*k3[3],   xp+dt*k3[4],   u_v)

            PA  += dt / 6.0 * (k1[0] + 2*k2[0] + 2*k3[0] + k4[0])
            PB  += dt / 6.0 * (k1[1] + 2*k2[1] + 2*k3[1] + k4[1])
            xv  += dt / 6.0 * (k1[2] + 2*k2[2] + 2*k3[2] + k4[2])
            vp  += dt / 6.0 * (k1[3] + 2*k2[3] + 2*k3[3] + k4[3])
            xp  += dt / 6.0 * (k1[4] + 2*k2[4] + 2*k3[4] + k4[4])

            # Physical bounds: pressures non-negative, spool limited
            PA = max(PA, 0.0)
            PB = max(PB, 0.0)
            xv = max(-1.0, min(1.0, xv))

        self.P_A, self.P_B, self.x_v, self.v_p, self.x_p = PA, PB, xv, vp, xp
        return self.force()

    # ------------------------------------------------------------------
    # Linearized 2-state model for LQR / MPC (at nominal operating point)
    # Used to build StateSpace object for ctrl_toolbox.
    # States: [F_delta (N), x_v (-)],  input: u_v (-),  output: F_delta (N)
    # ------------------------------------------------------------------
    def linear_model(self, params: dict):
        """
        Return (A_d, B_d, C_d, D_d) discretized with ZOH at Ts.
        Linearization at P_A = P_A0, P_B = P_B0, x_v=0, v_p=0.
        """
        import numpy as np
        try:
            import scipy.signal as sig
        except ImportError:
            return None

        P_A0 = params.get('P_A0', 3.0e6)
        P_B0 = params.get('P_B0', 5.0e6)

        # Approximate flow gain dQ/dx_v at x_v=0
        # Q_A approx = Kq * x_v * sqrt(P_S - P_A0)  for x_v > 0
        K_qa = self.Kq * math.sqrt(max(self.P_S - P_A0, 0.0))
        K_qb = self.Kq * math.sqrt(max(P_B0, 0.0))

        # dF/dt = K_F * x_v + (small piston velocity terms ignored)
        K_F = (self.A_A * self.beta / self.V_A * K_qa
               + self.A_B * self.beta / self.V_B * K_qb)

        # Continuous-time model: x = [F_delta, x_v]
        A_c = np.array([[0.0,  K_F],
                        [0.0, -1.0 / self.tau_v]])
        B_c = np.array([[0.0],
                        [1.0 / self.tau_v]])
        C   = np.array([[1.0, 0.0]])
        D   = np.array([[0.0]])

        A_d, B_d, C_d, D_d, _ = sig.cont2discrete(
            (A_c, B_c, C, D), self.Ts, method='zoh')
        return A_d, B_d, C_d, D_d
