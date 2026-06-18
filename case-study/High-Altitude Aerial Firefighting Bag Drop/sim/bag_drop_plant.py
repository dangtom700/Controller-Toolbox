"""
bag_drop_plant.py
3D translational trajectory model for a water-absorbing bag aerial drop.

States: x = [x_h (m), y_lat (m), z (m), vx (m/s), vy (m/s), vz (m/s)]
  x_h   - downrange position (positive forward, along aircraft heading)
  y_lat - lateral position (positive right)
  z     - altitude above ground (positive up)
  vx    - downrange velocity [m/s]
  vy    - lateral velocity [m/s]
  vz    - vertical velocity [m/s] (negative = falling)

Release conditions:
  position = (x0, y0, h_drop)
  velocity = (V_aircraft, 0, 0)

Forces acting on bag:
  Gravity:   [0, 0, -m*g]
  Aero drag: F_drag = -0.5 * rho * Cd * A_ref * |v_rel| * v_rel_hat
  v_rel = bag_velocity - wind_velocity

Wind model (per trajectory): constant vector (wx, wy, wz).
  For turbulent scenarios the caller samples independent wind per run.

Integration: 4th-order Runge-Kutta at Ts=0.05 s.

Reference:
  Sun et al. (2025) Results in Engineering 27, 105940.
  Bag treated as blunt body; Cd approx 0.8 (spherical cap approximation).
"""

import math


class BagDropPlant:
    def __init__(self, params: dict):
        self.m     = float(params['bag_mass_kg'])
        self.Cd    = float(params['Cd'])
        D          = float(params['bag_diameter_m'])
        self.A_ref = math.pi * (D / 2.0) ** 2   # frontal area [m^2]
        self.rho   = float(params.get('rho_air', 1.2))
        self.g     = float(params.get('g', 9.81))
        self.Ts    = float(params.get('Ts', 0.05))

    # ------------------------------------------------------------------
    def _drag_accel(self, vx, vy, vz, wx, wy, wz):
        """Aerodynamic drag acceleration components [m/s^2]."""
        vrx = vx - wx
        vry = vy - wy
        vrz = vz - wz
        v_mag = math.sqrt(vrx * vrx + vry * vry + vrz * vrz)
        if v_mag < 1.0e-8:
            return 0.0, 0.0, 0.0
        f_over_m = 0.5 * self.rho * self.Cd * self.A_ref * v_mag * v_mag / self.m
        return (-f_over_m * vrx / v_mag,
                -f_over_m * vry / v_mag,
                -f_over_m * vrz / v_mag)

    def _derivs(self, state, wx, wy, wz):
        x, y, z, vx, vy, vz = state
        ax, ay, az = self._drag_accel(vx, vy, vz, wx, wy, wz)
        return (vx, vy, vz, ax, ay, az - self.g)

    # ------------------------------------------------------------------
    def simulate(self, h_drop, V_aircraft, wind,
                 Ts=None, x0=0.0, y0=0.0):
        """
        Simulate bag from release to ground impact.

        Parameters
        ----------
        h_drop     : release altitude above ground [m]
        V_aircraft : aircraft speed at release [m/s]
        wind       : (wx, wy, wz) wind vector [m/s], constant during flight
        Ts         : integration step [s] (defaults to self.Ts)
        x0, y0     : release position offset [m]

        Returns
        -------
        x_impact : downrange position at ground [m]
        y_impact : lateral position at ground [m]
        t_flight : time of flight [s]
        traj     : list of (t, x, y, z) sampled every 5 steps
        """
        if Ts is None:
            Ts = self.Ts

        if len(wind) == 2:
            wx, wy, wz = wind[0], wind[1], 0.0
        else:
            wx, wy, wz = wind[0], wind[1], wind[2]

        state = [x0, y0, float(h_drop), float(V_aircraft), 0.0, 0.0]
        t = 0.0
        T_max = 120.0   # safety upper bound
        traj = []
        step_n = 0

        while state[2] > 0.0 and t < T_max:
            s = state
            k1 = self._derivs(s, wx, wy, wz)
            s2 = [s[i] + Ts / 2.0 * k1[i] for i in range(6)]
            k2 = self._derivs(s2, wx, wy, wz)
            s3 = [s[i] + Ts / 2.0 * k2[i] for i in range(6)]
            k3 = self._derivs(s3, wx, wy, wz)
            s4 = [s[i] + Ts * k3[i] for i in range(6)]
            k4 = self._derivs(s4, wx, wy, wz)

            state = [s[i] + Ts / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i])
                     for i in range(6)]
            t += Ts
            step_n += 1
            if step_n % 5 == 0:
                traj.append((t, state[0], state[1], state[2]))

        return state[0], state[1], t, traj

    # ------------------------------------------------------------------
    def nominal_impact(self, h_drop, V_aircraft, Ts=None):
        """Impact point with zero wind (reference for error computation)."""
        return self.simulate(h_drop, V_aircraft, (0.0, 0.0, 0.0), Ts=Ts)
