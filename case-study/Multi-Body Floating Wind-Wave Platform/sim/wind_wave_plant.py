"""
wind_wave_plant.py
Simplified 4-state FOWT + WEC coupled model.

States: x = [z, zdot, x_rel, xrel_dot]
  z        - FOWT heave displacement [m]
  zdot     - FOWT heave velocity [m/s]
  x_rel    - WEC arm displacement relative to platform [m]
  xrel_dot - WEC arm relative velocity [m/s]

Input:  F_PTO  - power take-off force [N]  (positive = resist WEC motion)
Output: (z, zdot, x_rel, xrel_dot)

Equations (continuous time):
  FOWT:  M_f*z_ddot  + B_f*zdot  + K_f*z     = F_wave(t)
  WEC:   M_w*xr_ddot + B_w*xr_dot + K_w*x_rel = gamma*F_wave(t) - F_PTO

Wave force:
  F_wave(t) = F_amp * sin(2*pi/T_wave * t)
  F_amp     = M_f * (2*pi/T_wave)^2 * (H_wave/2)   [inertia-dominated approximation]

Integration: 4th-order Runge-Kutta at Ts.
"""

import math


class FOWTWECPlant:
    def __init__(self, params: dict):
        self.M_f   = params['M_f']
        self.B_f   = params['B_f']
        self.K_f   = params['K_f']
        self.M_w   = params['M_w']
        self.B_w   = params['B_w']
        self.K_w   = params['K_w']
        self.gamma = params['gamma']
        self.F_max = params['F_max']
        self.Ts    = params['Ts']
        # State: [z, zdot, x_rel, xrel_dot]
        self._x = [0.0, 0.0, 0.0, 0.0]
        # Wave forcing function (set before stepping)
        self._wave_fn = lambda t: 0.0

    def set_wave(self, wave_fn):
        """Provide F_wave(t) callable."""
        self._wave_fn = wave_fn

    def _derivs(self, x, F_pto, t):
        z, zd, xr, xrd = x
        Fw = self._wave_fn(t)
        z_ddot  = (Fw - self.B_f * zd - self.K_f * z) / self.M_f
        xr_ddot = (self.gamma * Fw - F_pto - self.B_w * xrd - self.K_w * xr) / self.M_w
        return [zd, z_ddot, xrd, xr_ddot]

    def step(self, F_pto: float, t: float):
        """Advance one time step (RK4). Returns (z, zdot, x_rel, xrel_dot)."""
        dt     = self.Ts
        x      = self._x
        F_pto  = max(-self.F_max, min(self.F_max, F_pto))

        k1 = self._derivs(x,                             F_pto, t)
        k2 = self._derivs([xi + dt/2*ki for xi,ki in zip(x, k1)], F_pto, t + dt/2)
        k3 = self._derivs([xi + dt/2*ki for xi,ki in zip(x, k2)], F_pto, t + dt/2)
        k4 = self._derivs([xi + dt*ki  for xi,ki in zip(x, k3)], F_pto, t + dt)

        self._x = [xi + dt/6*(k1i + 2*k2i + 2*k3i + k4i)
                   for xi, k1i, k2i, k3i, k4i in zip(x, k1, k2, k3, k4)]
        return tuple(self._x)

    def reset(self):
        self._x = [0.0, 0.0, 0.0, 0.0]

    def state(self):
        return tuple(self._x)

    def power(self, F_pto: float) -> float:
        """Instantaneous extracted power P = F_PTO * xrel_dot [W]."""
        return F_pto * self._x[3]


def make_wave_fn(scenario: dict, plant_params: dict):
    """Build and return F_wave(t) callable for the given scenario."""
    M_f    = plant_params['M_f']
    H      = scenario['H_wave']
    A_wave = H / 2.0   # wave amplitude [m]

    wave_type = scenario['wave_type']

    if wave_type == 'regular':
        T_w  = scenario['T_wave']
        omega_w = 2.0 * math.pi / T_w
        # Inertia-dominated Froude-Krylov force approximation
        F_amp = M_f * omega_w ** 2 * A_wave
        def wave_fn(t):
            return F_amp * math.sin(omega_w * t)
        return wave_fn

    elif wave_type == 'bichromatic':
        T1 = scenario['T_wave'];  H1 = scenario['H_wave']
        T2 = scenario['T_wave2']; H2 = scenario['H_wave2']
        o1 = 2.0 * math.pi / T1;  o2 = 2.0 * math.pi / T2
        F1 = M_f * o1**2 * (H1/2)
        F2 = M_f * o2**2 * (H2/2)
        def wave_fn(t):
            return F1 * math.sin(o1 * t) + F2 * math.sin(o2 * t)
        return wave_fn

    elif wave_type == 'step_freq':
        T1   = scenario['T_wave_1']; T2 = scenario['T_wave_2']
        t_sw = scenario['step_time']
        o1   = 2.0 * math.pi / T1;  o2 = 2.0 * math.pi / T2
        F1   = M_f * o1**2 * (H/2)
        F2   = M_f * o2**2 * (H/2)
        def wave_fn(t):
            if t < t_sw:
                return F1 * math.sin(o1 * t)
            # Keep phase continuous at transition
            phase_at_sw = o1 * t_sw
            return F2 * math.sin(o2 * (t - t_sw) + phase_at_sw)
        return wave_fn

    # Fallback
    return lambda t: 0.0
