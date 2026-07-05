"""
satellite_launch_vehicle_systems_plant.py
Rigid-body pitch-plane attitude model of a Satellite Launch Vehicle (SLV)
during the atmospheric phase, after Nair, Selvaganesan & Lalithambika (2016),
"Lyapunov based PD/PID in model reference adaptive control for satellite launch
vehicle systems", Aerospace Science and Technology 51, 70-77.

Dynamics (paper Eq. 1-3, wind retained):
    theta_ddot = mu_alpha * (theta + alpha_w) + mu_c * delta
with the single-plane rigid-body reduction (short-period, drift term z_dot/V
neglected because V is large in the high-dynamic-pressure regime, paper Sec. 2):

    theta            pitch attitude angle [rad]            (controlled output)
    theta_dot        pitch rate           [rad/s]
    delta            thrust-deflection (gimbal) angle [rad] (control input, |delta| <= delta_max = 8 deg)
    alpha_w = -Vw/V  wind angle of attack [rad]            (disturbance)

    mu_alpha = L_alpha l_alpha / I   aerodynamic-moment coefficient [1/s^2]
    mu_c     = T_c l_c / I           control-moment coefficient     [1/s^2]

Both mu_alpha and mu_c are TIME VARYING during ascent (thrust, mass, inertia and
dynamic pressure all change - paper Remark 1 / Fig. 3). mu_alpha > 0 makes the
open-loop plant UNSTABLE (a pole at +sqrt(mu_alpha), paper Eq. 2), peaking in the
transonic regime (~40-60 s).

Model simplifications (documented per project convention, see README):
  1. The paper's Fig. 3 gives mu_c(t), mu_alpha(t) graphically only; numeric
     values are not recoverable from the PDF text. This model uses a smooth,
     physically-representative reconstruction: mu_alpha a Gaussian bump peaking in
     the transonic regime, mu_c a slow linear decay as propellant depletes. The
     qualitative behaviour the paper exercises (an unstable, time-varying plant
     whose instability is worst in the transonic band) is preserved; the profile
     parameters live in config/plant_params.json so they are queryable, not baked in.
  2. Drift/force equation (z_dot/V) is dropped exactly as the paper does for the
     high-velocity aerodynamic phase (Sec. 2), leaving the theta/delta 2nd-order
     rigid-body pair.
  3. Second-order actuator dynamics + slew/position limits (paper Sec. 4.1
     assumption 2) are reduced to a single position limit |delta| <= delta_max; the
     rate limit is applied to the commanded gimbal in simulation_runner.py.
  4. Open-loop divergence is bounded by clamping |theta| <= theta_clamp and
     |theta_dot| <= rate_clamp (vehicle-breakup / tumble saturation), so the
     uncontrolled OpenLoop baseline yields a finite (large) IAE instead of an
     inf/NaN overflow over the 100 s run.

Integration: classical RK4 at Ts.
"""

import math


class SLVPlant:
    """SLV pitch-plane attitude plant: x = [theta, theta_dot]."""

    def __init__(self, params: dict):
        p = params
        self.Ts = p.get("Ts", 0.05)

        # Time-varying aerodynamic-moment coefficient mu_alpha(t) [1/s^2]:
        # Gaussian bump peaking in the transonic regime.
        self.mu_alpha_base = p.get("mu_alpha_base", 0.8)
        self.mu_alpha_peak = p.get("mu_alpha_peak", 4.0)
        self.mu_alpha_tpeak = p.get("mu_alpha_tpeak", 50.0)
        self.mu_alpha_width = p.get("mu_alpha_width", 18.0)

        # Time-varying control-moment coefficient mu_c(t) [1/s^2]:
        # slow linear decay (thrust/inertia change as propellant depletes).
        self.mu_c0 = p.get("mu_c0", 9.0)
        self.mu_c_decay = p.get("mu_c_decay", 0.35)
        self.mu_c_Tref = p.get("mu_c_Tref", 100.0)

        self.delta_max = p.get("delta_max", 0.13963)   # 8 deg in rad
        self.theta_clamp = p.get("theta_clamp", 1.0)   # rad (~57 deg)
        self.rate_clamp = p.get("rate_clamp", 5.0)     # rad/s

        theta0 = p.get("theta0", 0.0)
        theta_dot0 = p.get("theta_dot0", 0.0)
        self._x = [theta0, theta_dot0]

        # Wind disturbance function alpha_w(t) [rad]; default: no wind.
        self._wind_fn = lambda t: 0.0

    # -- time-varying parameters ------------------------------------------
    def mu_alpha(self, t: float) -> float:
        z = (t - self.mu_alpha_tpeak) / self.mu_alpha_width
        return self.mu_alpha_base + self.mu_alpha_peak * math.exp(-z * z)

    def mu_c(self, t: float) -> float:
        return self.mu_c0 * (1.0 - self.mu_c_decay * t / self.mu_c_Tref)

    def set_wind_fn(self, wind_fn):
        """wind_fn(t) -> alpha_w [rad]."""
        self._wind_fn = wind_fn

    def reset(self, x0=None):
        self._x = [0.0, 0.0] if x0 is None else list(x0)

    def state(self):
        return tuple(self._x)

    def output(self) -> float:
        return self._x[0]

    # -- dynamics ----------------------------------------------------------
    def _derivs(self, x, delta, t):
        theta, theta_dot = x
        alpha_w = self._wind_fn(t)
        theta_ddot = self.mu_alpha(t) * (theta + alpha_w) + self.mu_c(t) * delta
        return [theta_dot, theta_ddot]

    def step(self, delta: float, t: float):
        """Advance one Ts step (RK4). delta = commanded gimbal angle [rad],
        saturated to +/- delta_max. Returns the new state tuple."""
        d = max(-self.delta_max, min(self.delta_max, delta))
        dt = self.Ts
        x = self._x
        k1 = self._derivs(x, d, t)
        x2 = [xi + dt / 2 * ki for xi, ki in zip(x, k1)]
        k2 = self._derivs(x2, d, t + dt / 2)
        x3 = [xi + dt / 2 * ki for xi, ki in zip(x, k2)]
        k3 = self._derivs(x3, d, t + dt / 2)
        x4 = [xi + dt * ki for xi, ki in zip(x, k3)]
        k4 = self._derivs(x4, d, t + dt)
        new_x = [xi + dt / 6 * (a + 2 * b + 2 * c + e)
                 for xi, a, b, c, e in zip(x, k1, k2, k3, k4)]
        # Breakup/tumble saturation -> keeps open-loop divergence finite.
        new_x[0] = max(-self.theta_clamp, min(self.theta_clamp, new_x[0]))
        new_x[1] = max(-self.rate_clamp, min(self.rate_clamp, new_x[1]))
        self._x = new_x
        return tuple(self._x)


def make_wind_fn(scenario: dict):
    """Build alpha_w(t) [rad] for the given scenario.

    Wind profiles:
      none  - no wind (default).
      gust  - a raised-cosine gust of amplitude alpha_w_max [rad] centred at
              t_gust with half-width w_gust [s] (paper Sec. 4.2.2 synthetic
              wind profile injected with the command held at zero).
    """
    wind = scenario.get("wind", {"type": "none"})
    wtype = wind.get("type", "none")

    if wtype == "gust":
        amp = wind.get("alpha_w_max", 0.05)
        t0 = wind.get("t_gust", 45.0)
        w = wind.get("w_gust", 8.0)

        def wind_fn(t):
            dt = t - t0
            if abs(dt) > w:
                return 0.0
            return amp * 0.5 * (1.0 + math.cos(math.pi * dt / w))
        return wind_fn

    return lambda t: 0.0
