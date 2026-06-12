"""
3-DOF MMG manoeuvring plant model.
Source: Meng et al. (2025), Ocean Engineering 321, 120432, Table 5.

State vector: [u, v, r, psi, x, y]
  u   surge velocity [m/s]
  v   sway velocity  [m/s]
  r   yaw rate       [rad/s]
  psi heading        [rad]
  x   x-position     [m]
  y   y-position     [m]

Input vector: [n_rps, delta_rad]
  n_rps    propeller speed [rev/s]  (max 10.7 = 642 RPM)
  delta_rad rudder angle   [rad]    (max +-35 deg = +-0.6109 rad)

Equations (continuous form derived from Eq. 3, 4 of the paper):
  du/dt = a1*u^2 + a2*v*r + a3*v^2 + a4*r^2 + a5*n^2 + a6*delta + du
  dv/dt = b1*v + b2*r + b3*|v|*v + b4*|r|*r + b5*|v|*r + b6*(-u*r) + b7*delta + dv
  dr/dt = c1*v + c2*r + c3*|v|*v + c4*v^2*r + c5*v*r^2 + c6*delta + dr
  dpsi/dt = r
  dx/dt  = u*cos(psi) - v*sin(psi)
  dy/dt  = u*sin(psi) + v*cos(psi)
"""

import math


class ShipPlant:
    def __init__(self, params):
        p = params["identified_params"]
        self.a1 = p["a1"]; self.a2 = p["a2"]; self.a3 = p["a3"]
        self.a4 = p["a4"]; self.a5 = p["a5"]; self.a6 = p["a6"]
        self.b1 = p["b1"]; self.b2 = p["b2"]; self.b3 = p["b3"]
        self.b4 = p["b4"]; self.b5 = p["b5"]; self.b6 = p["b6"]
        self.b7 = p["b7"]
        self.c1 = p["c1"]; self.c2 = p["c2"]; self.c3 = p["c3"]
        self.c4 = p["c4"]; self.c5 = p["c5"]; self.c6 = p["c6"]

        lim = params["actuator_limits"]
        self.n_max = lim["n_max_rps"]
        self.n_min = lim["n_min_rps"]
        self.delta_max = lim["delta_max_rad"]
        self.delta_rate_max = lim["delta_rate_max_rad_s"]

        self.Ts = params["integration"]["Ts"]

        ic = params["initial_state"]
        self._x0 = [ic["u0"], ic["v0"], ic["r0"], ic["psi0"], ic["x0"], ic["y0"]]
        self._x = list(self._x0)
        self._delta_prev = 0.0
        self._disturbance_fn = None

    def reset(self, x0=None):
        self._x = list(x0 if x0 is not None else self._x0)
        self._delta_prev = 0.0

    def state(self):
        return list(self._x)

    def set_disturbance_fn(self, fn):
        self._disturbance_fn = fn

    def _Fu(self, u, v, r):
        return self.a1*u*u + self.a2*v*r + self.a3*v*v + self.a4*r*r

    def _Fv(self, u, v, r):
        return (self.b1*v + self.b2*r + self.b3*abs(v)*v +
                self.b4*abs(r)*r + self.b5*abs(v)*r + self.b6*(-u*r))

    def _Fr(self, u, v, r):
        return (self.c1*v + self.c2*r + self.c3*abs(v)*v +
                self.c4*v*v*r + self.c5*v*r*r)

    def _derivs(self, x, u_in, t):
        u, v, r, psi, _xp, _yp = x
        n_rps, delta_rad = u_in

        d_u = d_v = d_r = 0.0
        if self._disturbance_fn is not None:
            d_u, d_v, d_r = self._disturbance_fn(t)

        du_dt = self._Fu(u, v, r) + self.a5*n_rps*n_rps + self.a6*delta_rad + d_u
        dv_dt = self._Fv(u, v, r) + self.b7*delta_rad + d_v
        dr_dt = self._Fr(u, v, r) + self.c6*delta_rad + d_r
        dpsi_dt = r
        dx_dt = u*math.cos(psi) - v*math.sin(psi)
        dy_dt = u*math.sin(psi) + v*math.cos(psi)
        return [du_dt, dv_dt, dr_dt, dpsi_dt, dx_dt, dy_dt]

    def step(self, n_rps_cmd, delta_rad_cmd, t):
        # Rate-limit rudder
        rate = (delta_rad_cmd - self._delta_prev) / self.Ts
        rate = max(-self.delta_rate_max, min(self.delta_rate_max, rate))
        delta_rad = self._delta_prev + rate * self.Ts

        # Saturate both actuators
        n_rps = max(self.n_min, min(self.n_max, n_rps_cmd))
        delta_rad = max(-self.delta_max, min(self.delta_max, delta_rad))
        self._delta_prev = delta_rad

        # RK4 integration
        u_in = (n_rps, delta_rad)
        x = self._x
        Ts = self.Ts
        k1 = self._derivs(x, u_in, t)
        k2 = self._derivs([xi + Ts/2*ki for xi, ki in zip(x, k1)], u_in, t + Ts/2)
        k3 = self._derivs([xi + Ts/2*ki for xi, ki in zip(x, k2)], u_in, t + Ts/2)
        k4 = self._derivs([xi + Ts*ki for xi, ki in zip(x, k3)], u_in, t + Ts)
        self._x = [xi + Ts/6*(k1i + 2*k2i + 2*k3i + k4i)
                   for xi, k1i, k2i, k3i, k4i in zip(x, k1, k2, k3, k4)]
        return n_rps, delta_rad

    # Expose model functions for controllers that need plant knowledge
    def Fu(self, u, v, r): return self._Fu(u, v, r)
    def Fv(self, u, v, r): return self._Fv(u, v, r)
    def Fr(self, u, v, r): return self._Fr(u, v, r)


def make_disturbance_fn(scenario, plant_params):
    scale = scenario.get("disturbance_scale", 0.0)
    d = plant_params["disturbance"]
    def fn(t):
        du = scale * d["d_u_amp"] * math.sin(d["d_u_freq"] * t)
        dv = scale * d["d_v_amp"] * math.cos(d["d_v_freq"] * t)
        dr = scale * d["d_r_amp"] * math.cos(d["d_r_freq"] * t)
        return du, dv, dr
    return fn


def make_ref_fn(scenario):
    """Return callable (t) -> (xd, yd, xd_dot, yd_dot, xd_ddot, yd_ddot)."""
    traj = scenario.get("trajectory", "sine")

    if traj == "sine":
        A = scenario.get("xd_amp", 20.0)
        om = scenario.get("xd_omega", 0.07)
        vy = scenario.get("yd_speed", 1.0)
        def fn(t):
            xd = A * math.sin(om * t)
            yd = vy * t
            xd_d = A * om * math.cos(om * t)
            yd_d = vy
            xd_dd = -A * om * om * math.sin(om * t)
            yd_dd = 0.0
            return xd, yd, xd_d, yd_d, xd_dd, yd_dd

    elif traj == "straight":
        vy = scenario.get("yd_speed", 2.0)
        def fn(t):
            return 0.0, vy*t, 0.0, vy, 0.0, 0.0

    elif traj == "circle":
        R = scenario.get("radius", 25.0)
        om = scenario.get("omega", 0.06)
        # Centre at (R, 0) so that at t=0 the path starts near (0,0)
        def fn(t):
            xd = R * (math.cos(om * t) - 1.0)
            yd = R * math.sin(om * t)
            xd_d = -R * om * math.sin(om * t)
            yd_d =  R * om * math.cos(om * t)
            xd_dd = -R * om * om * math.cos(om * t)
            yd_dd = -R * om * om * math.sin(om * t)
            return xd, yd, xd_d, yd_d, xd_dd, yd_dd

    elif traj == "zigzag":
        A = scenario.get("xd_amp", 20.0)
        T = scenario.get("period", 30.0)
        vy = scenario.get("yd_speed", 1.5)
        om = math.pi / T
        def fn(t):
            xd = A * math.sin(om * t)
            yd = vy * t
            xd_d = A * om * math.cos(om * t)
            yd_d = vy
            xd_dd = -A * om * om * math.sin(om * t)
            yd_dd = 0.0
            return xd, yd, xd_d, yd_d, xd_dd, yd_dd

    else:
        def fn(t):
            return 0.0, t, 0.0, 1.0, 0.0, 0.0

    return fn
