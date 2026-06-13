"""
road_model.py
Road profile generator with per-axle time-delay ring buffer.

Only 2 independent road channels are generated (left track, right track).
The 6 wheel contact inputs are derived by delaying the front-axle signal
by the axle-to-axle transit time at vehicle speed v_vehicle [m/s]:

  delay_mid  = L1 / v_vehicle        # front -> middle axle
  delay_rear = (L1+L2) / v_vehicle   # front -> rear axle

Returns z_r[6] = [rF, rM, rR, lF, lM, lR] (right front/mid/rear, then left).
"""

import numpy as np
from collections import deque


class RoadProfile:
    """
    Road profile with time-delay ring buffer.

    Parameters
    ----------
    profile_id : str
        One of 's01_bump', 's02_resonance', 's03_rough',
        's04_speedbump', 's05_comfort'
    scenario : dict
        Scenario config loaded from JSON (provides profile params).
    Ts : float
        Simulation sample time [s].
    v_vehicle : float
        Vehicle speed [m/s]; used to compute axle transit delays.
    L1 : float
        Front-to-middle axle distance [m].
    L2 : float
        Middle-to-rear axle distance [m].
    """

    def __init__(self, scenario: dict, Ts: float,
                 v_vehicle: float = 22.2, L1: float = 2.0, L2: float = 2.0):
        self._Ts    = Ts
        self._v     = v_vehicle
        self._L1    = L1
        self._L2    = L2
        self._id    = scenario.get('id', 'unknown')
        self._cfg   = scenario

        # Compute delay in samples (at least 1 sample)
        delay_mid_s  = max(1, round(L1 / v_vehicle / Ts))
        delay_rear_s = max(1, round((L1 + L2) / v_vehicle / Ts))

        # Ring buffers for right and left channels
        self._buf_R = deque([0.0] * (delay_rear_s + 1), maxlen=delay_rear_s + 1)
        self._buf_L = deque([0.0] * (delay_rear_s + 1), maxlen=delay_rear_s + 1)
        self._delay_mid  = delay_mid_s
        self._delay_rear = delay_rear_s

        # Profile-specific state
        self._t       = 0.0
        self._rng     = np.random.default_rng(42)
        self._rough_phase_R = self._rng.uniform(0, 2 * np.pi)
        self._rough_phase_L = self._rng.uniform(0, 2 * np.pi)
        self._comfort_f  = 1.0   # sweep start frequency [Hz]

    def reset(self):
        self._t = 0.0
        n = self._delay_rear + 1
        self._buf_R = deque([0.0] * n, maxlen=n)
        self._buf_L = deque([0.0] * n, maxlen=n)
        self._comfort_f = 1.0

    def step(self) -> np.ndarray:
        """
        Advance one Ts and return road inputs z_r[6]:
          [rF, rM, rR, lF, lM, lR] in metres.
        """
        t = self._t
        cfg = self._cfg
        pid = self._id

        # --- Generate fresh front-axle signal ---
        if pid == 's01_bump':
            amp   = cfg.get('amp', 0.025)         # 25 mm step bump
            t_hit = cfg.get('t_hit', 0.5)         # bump onset [s]
            zR = amp if t >= t_hit else 0.0
            zL = amp if t >= t_hit else 0.0

        elif pid == 's02_resonance':
            amp = cfg.get('amp', 0.015)           # 15 mm
            f   = cfg.get('freq', 1.3)            # body resonance ~ 1.3 Hz
            zR = amp * np.sin(2 * np.pi * f * t)
            zL = amp * np.sin(2 * np.pi * f * t)

        elif pid == 's03_rough':
            amp = cfg.get('amp', 0.005)           # 5 mm RMS
            f   = cfg.get('freq', 8.0)            # 8 Hz high-frequency
            zR = amp * np.sin(2 * np.pi * f * t + self._rough_phase_R)
            zL = amp * np.sin(2 * np.pi * f * t + self._rough_phase_L)

        elif pid == 's04_speedbump':
            # Versine speed bump: L_bump [m], h_bump [m], centred at t_hit
            h   = cfg.get('h', 0.050)             # 50 mm
            L_b = cfg.get('L_bump', 0.5)          # 0.5 m length
            t_c = cfg.get('t_hit', 0.5)
            x_c = self._v * (t - t_c)             # position relative to bump centre
            if abs(x_c) <= L_b / 2:
                zR = h / 2 * (1 - np.cos(2 * np.pi * x_c / L_b))
                zL = zR
            else:
                zR = zL = 0.0

        elif pid == 's05_comfort':
            # ISO 2631 comfort: sine sweep 1 → 10 Hz over T_sweep seconds
            amp    = cfg.get('amp', 0.010)         # 10 mm
            f_end  = cfg.get('f_end', 10.0)
            f_rate = cfg.get('f_rate', 0.02)       # Hz / s
            f      = min(self._comfort_f, f_end)
            self._comfort_f += f_rate * self._Ts
            zR = amp * np.sin(2 * np.pi * f * t)
            zL = amp * np.sin(2 * np.pi * f * t)

        else:
            zR = zL = 0.0

        # Push fresh values into ring buffers (front of deque = newest)
        self._buf_R.appendleft(zR)
        self._buf_L.appendleft(zL)

        # Sample at delay indices (0 = current = front axle)
        rF = self._buf_R[0]
        rM = self._buf_R[min(self._delay_mid,  len(self._buf_R) - 1)]
        rR = self._buf_R[min(self._delay_rear, len(self._buf_R) - 1)]
        lF = self._buf_L[0]
        lM = self._buf_L[min(self._delay_mid,  len(self._buf_L) - 1)]
        lR = self._buf_L[min(self._delay_rear, len(self._buf_L) - 1)]

        self._t += self._Ts
        return np.array([rF, rM, rR, lF, lM, lR])
