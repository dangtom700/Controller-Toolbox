"""
ev6x6_plant.py
40-state 6x6 EV active suspension plant.

State: x = [q; q_dot] where q in R^20 contains:
  q[0]   = Z      body vertical displacement [m]
  q[1]   = theta  body pitch angle [rad]
  q[2]   = phi    body roll angle [rad]
  q[3]   = Z_wr1  right-front wheel vertical [m]
  q[4]   = Z_wr2  right-middle wheel vertical [m]
  q[5]   = Z_wr3  right-rear wheel vertical [m]
  q[6]   = Z_wl1  left-front wheel vertical [m]
  q[7]   = Z_wl2  left-middle wheel vertical [m]
  q[8]   = Z_wl3  left-rear wheel vertical [m]
  q[9]   = Z_admr1 right-front in-wheel motor [m]
  q[10]  = Z_admr2 right-middle in-wheel motor [m]
  q[11]  = Z_admr3 right-rear in-wheel motor [m]
  q[12]  = Z_adml1 left-front in-wheel motor [m]
  q[13]  = Z_adml2 left-middle in-wheel motor [m]
  q[14]  = Z_adml3 left-rear in-wheel motor [m]
  q[15]  = Z_seat  seat vertical [m]
  q[16]  = Z_p     pelvis [m]
  q[17]  = Z_lt    lower torso [m]
  q[18]  = Z_ut    upper torso [m]
  q[19]  = Z_h     head [m]

Inputs:
  F_act[0:6]  - active suspension forces at 6 wheel corners [N]
  z_r[0:6]    - road surface heights at 6 wheel contact patches [m]

Integration: RK4 at Ts (default 0.005 s)
"""

import numpy as np
from scipy.signal import cont2discrete


# Default parameters (Aydogan & Yildiz 2025 Table 2 and standard quarter-car values)
DEFAULT_PARAMS = {
    # Body
    'M'   : 1600.0,    # body mass [kg] (15-DOF vehicle body sprung mass)
    'I_x' : 600.0,     # roll moment of inertia [kg.m^2]
    'I_y' : 1200.0,    # pitch moment of inertia [kg.m^2]

    # Geometry [m]  (6-wheel arrangement: front, middle, rear)
    'a'   : [1.5,  0.0, -1.5],   # longitudinal offset per axle [m] (+ = front)
    'W'   : 1.8,                   # track width [m] (half-track = W/2 = 0.9 m)

    # Per-corner suspension (k_s, c_s same for all 6 corners)
    'k_s' : 16000.0,   # suspension spring stiffness [N/m]
    'c_s' : 980.0,     # suspension damper coefficient [N.s/m]
    'k_t' : 160000.0,  # tyre spring stiffness [N/m] (tyre damping neglected)
    'c_t' : 0.0,       # tyre damping [N.s/m]

    # In-wheel motor (actuated drive motor mass on each corner)
    'M_w'   : 36.0,    # wheel assembly mass [kg]
    'M_adm' : 50.0,    # in-wheel motor mass [kg]
    'k_adm' : 120000.0, # motor mount stiffness [N/m]
    'c_adm' : 500.0,    # motor mount damping [N.s/m]

    # Seat (connects body to human via spring-damper)
    'M_seat' : 35.0,   # seat mass [kg]
    'k_seat' : 18000.0, # seat stiffness [N/m]
    'c_seat' : 650.0,   # seat damping [N.s/m]

    # Human biodynamic model (ISO 2631 Guide E, 5-DOF)
    'M_p'  : 10.0,   # pelvis mass [kg]
    'M_lt' : 20.0,   # lower torso [kg]
    'M_ut' : 25.0,   # upper torso [kg]
    'M_h'  : 5.0,    # head [kg]

    'k1' : 127000.0, 'c1' : 740.0,   # seat-pelvis (k1, c1)
    'k2' : 21000.0,  'c2' : 1920.0,  # pelvis-lower torso
    'k3' : 21000.0,  'c3' : 1920.0,  # lower-upper torso
    'k4' : 10000.0,  'c4' : 400.0,   # upper torso-head
    'k5' : 18000.0,  'c5' : 650.0,   # seat (body-seat mount; same as k_seat/c_seat)

    # Actuator limits
    'F_max' : 3000.0,  # max active force per corner [N]

    # Simulation
    'Ts' : 0.005,      # sample time [s]
}

N_DOF  = 20   # degrees of freedom
N_STAT = 40   # state size


def _assemble_matrices(p: dict):
    """
    Build M_mat (diagonal), K_mat, C_mat, B_u, B_f for the 20-DOF system.
    Returns (M_mat, K_mat, C_mat, B_u, B_f) all np.ndarray (20,20) or (20,6).
    """
    a  = p['a']          # [a1, a2, a3] longitudinal offsets [m]
    hw = p['W'] / 2.0    # half track width [m]

    M_mat = np.zeros((N_DOF, N_DOF))
    # Body (DOFs 0-2)
    M_mat[0, 0] = p['M']
    M_mat[1, 1] = p['I_y']
    M_mat[2, 2] = p['I_x']
    # Wheels (DOFs 3-8: right front/mid/rear, left front/mid/rear)
    for i in range(6):
        M_mat[3 + i, 3 + i] = p['M_w']
    # In-wheel motors (DOFs 9-14)
    for i in range(6):
        M_mat[9 + i, 9 + i] = p['M_adm']
    # Human (DOFs 15-19)
    M_mat[15, 15] = p['M_seat']
    M_mat[16, 16] = p['M_p']
    M_mat[17, 17] = p['M_lt']
    M_mat[18, 18] = p['M_ut']
    M_mat[19, 19] = p['M_h']

    K_mat = np.zeros((N_DOF, N_DOF))
    C_mat = np.zeros((N_DOF, N_DOF))
    B_u   = np.zeros((N_DOF, 6))   # road input (tyre stiffness coupling)
    B_f   = np.zeros((N_DOF, 6))   # active force input

    # Corner indices: [rF, rM, rR, lF, lM, lR]  (right +hw, left -hw)
    # Wheel DOF  = 3 + corner_idx
    # Motor DOF  = 9 + corner_idx
    sign_lat  = [+1, +1, +1, -1, -1, -1]   # right corners: +e, left: -e
    axle_idx  = [ 0,  1,  2,  0,  1,  2]   # maps corner to axle (a[0..2])

    ks = p['k_s']; cs = p['c_s']
    kt = p['k_t']; ct = p['c_t']
    ka = p['k_adm']; ca = p['c_adm']

    for ci in range(6):
        ai  = a[axle_idx[ci]]        # longitudinal offset
        ei  = sign_lat[ci] * hw      # lateral offset (+ right, - left)
        wdof = 3 + ci                # wheel DOF index
        mdof = 9 + ci                # motor DOF index

        # ---- Body-wheel suspension coupling ----
        # Forces on body DOF 0 (Z): k_s*(Z_wi - Z)
        K_mat[0, 0]       += ks;  K_mat[0, wdof]   -= ks
        K_mat[wdof, 0]    -= ks;  K_mat[wdof, wdof] += ks

        # Forces on body DOF 1 (theta, pitch): k_s * a_i * (Z + a_i*theta - e_i*phi - Z_wi)
        K_mat[1, 1]       += ks * ai * ai
        K_mat[1, 0]       += ks * ai;    K_mat[0, 1]       += ks * ai
        K_mat[1, 2]       -= ks * ai * ei; K_mat[2, 1]     -= ks * ai * ei
        K_mat[1, wdof]    -= ks * ai;    K_mat[wdof, 1]    -= ks * ai

        # Forces on body DOF 2 (phi, roll): k_s * e_i * (Z + a_i*theta - e_i*phi - Z_wi)
        # Note: roll equation uses -e_i (restoring)
        K_mat[2, 2]       += ks * ei * ei
        K_mat[2, 0]       -= ks * ei;    K_mat[0, 2]       -= ks * ei
        K_mat[2, 1]       += ks * ai * ei  # already added via [1,2] symmetry
        K_mat[2, wdof]    += ks * ei;    K_mat[wdof, 2]    += ks * ei

        # Damping (same structure as K)
        C_mat[0, 0]       += cs;  C_mat[0, wdof]   -= cs
        C_mat[wdof, 0]    -= cs;  C_mat[wdof, wdof] += cs

        C_mat[1, 1]       += cs * ai * ai
        C_mat[1, 0]       += cs * ai;    C_mat[0, 1]       += cs * ai
        C_mat[1, 2]       -= cs * ai * ei; C_mat[2, 1]     -= cs * ai * ei
        C_mat[1, wdof]    -= cs * ai;    C_mat[wdof, 1]    -= cs * ai

        C_mat[2, 2]       += cs * ei * ei
        C_mat[2, 0]       -= cs * ei;    C_mat[0, 2]       -= cs * ei
        C_mat[2, 1]       += cs * ai * ei
        C_mat[2, wdof]    += cs * ei;    C_mat[wdof, 2]    += cs * ei

        # ---- Wheel-motor coupling (motor mount) ----
        K_mat[wdof, wdof] += ka;  K_mat[wdof, mdof] -= ka
        K_mat[mdof, wdof] -= ka;  K_mat[mdof, mdof] += ka
        C_mat[wdof, wdof] += ca;  C_mat[wdof, mdof] -= ca
        C_mat[mdof, wdof] -= ca;  C_mat[mdof, mdof] += ca

        # ---- Tyre: wheel-road coupling ----
        # Road input: k_t * z_r[ci] appears in wheel DOF equation
        K_mat[wdof, wdof] += kt
        B_u[wdof, ci]      = kt   # road force k_t * z_r into wheel eq
        if ct > 0:
            C_mat[wdof, wdof] += ct
            # road damping term handled in deriv via road velocity (assumed 0 here)

        # ---- Active actuator: force between wheel and body ----
        # F_act[ci] in wheel eq: -1 (reaction on wheel), +1 on body
        B_f[0,    ci] = +1.0   # body Z (vertical)
        B_f[1,    ci] = +ai    # body pitch
        B_f[2,    ci] = -ei    # body roll (force at +e pushes roll back)
        B_f[wdof, ci] = -1.0   # wheel (reaction)

    # ---- Seat-body coupling (DOF 15 = Z_seat) ----
    ks5 = p['k5']; cs5 = p['c5']
    K_mat[0,  15] -= ks5; K_mat[15,  0] -= ks5
    K_mat[0,   0] += ks5; K_mat[15, 15] += ks5
    C_mat[0,  15] -= cs5; C_mat[15,  0] -= cs5
    C_mat[0,   0] += cs5; C_mat[15, 15] += cs5

    # Seat-pelvis (15->16)
    k1=p['k1']; c1=p['c1']
    K_mat[15,15]+=k1; K_mat[15,16]-=k1; K_mat[16,15]-=k1; K_mat[16,16]+=k1
    C_mat[15,15]+=c1; C_mat[15,16]-=c1; C_mat[16,15]-=c1; C_mat[16,16]+=c1

    # Pelvis-lower torso (16->17)
    k2=p['k2']; c2=p['c2']
    K_mat[16,16]+=k2; K_mat[16,17]-=k2; K_mat[17,16]-=k2; K_mat[17,17]+=k2
    C_mat[16,16]+=c2; C_mat[16,17]-=c2; C_mat[17,16]-=c2; C_mat[17,17]+=c2

    # Lower-upper torso (17->18)
    k3=p['k3']; c3=p['c3']
    K_mat[17,17]+=k3; K_mat[17,18]-=k3; K_mat[18,17]-=k3; K_mat[18,18]+=k3
    C_mat[17,17]+=c3; C_mat[17,18]-=c3; C_mat[18,17]-=c3; C_mat[18,18]+=c3

    # Upper torso-head (18->19)
    k4=p['k4']; c4=p['c4']
    K_mat[18,18]+=k4; K_mat[18,19]-=k4; K_mat[19,18]-=k4; K_mat[19,19]+=k4
    C_mat[18,18]+=c4; C_mat[18,19]-=c4; C_mat[19,18]-=c4; C_mat[19,19]+=c4

    return M_mat, K_mat, C_mat, B_u, B_f


class EV6x6Plant:
    """40-state 6x6 EV active suspension plant."""

    def __init__(self, params: dict = None):
        p = dict(DEFAULT_PARAMS)
        if params:
            p.update(params)
        self._p = p
        self._Ts = p['Ts']

        M_mat, K_mat, C_mat, B_u, B_f = _assemble_matrices(p)

        # Store inverse mass matrix
        self._Minv = np.linalg.inv(M_mat)

        # Continuous state-space matrices (40x40 A, 40x6 B_road, 40x6 B_act)
        n = N_DOF
        self._A_c = np.block([
            [np.zeros((n, n)),     np.eye(n)             ],
            [-self._Minv @ K_mat, -self._Minv @ C_mat    ]
        ])
        self._B_road_c = np.vstack([
            np.zeros((n, 6)),
            self._Minv @ B_u
        ])
        self._B_act_c = np.vstack([
            np.zeros((n, 6)),
            self._Minv @ B_f
        ])

        # Discretise A + B_act via ZOH (B_road treated the same way)
        # Use scipy cont2discrete
        eye40 = np.eye(N_STAT)
        A_full = self._A_c
        B_combined = np.hstack([self._B_act_c, self._B_road_c])  # (40, 12)
        C_out = eye40
        D_out = np.zeros((N_STAT, 12))
        Ad, Bd_all, _, _, _ = cont2discrete(
            (A_full, B_combined, C_out, D_out), self._Ts, method='zoh')
        self._Ad      = Ad
        self._Bd_act  = Bd_all[:, :6]
        self._Bd_road = Bd_all[:, 6:]

        # State
        self._x = np.zeros(N_STAT)
        self._F_max = p['F_max']

    def step(self, F_act: np.ndarray, z_r: np.ndarray) -> np.ndarray:
        """Advance one Ts step (ZOH discrete); returns new state."""
        F_clamped = np.clip(F_act, -self._F_max, self._F_max)
        self._x = (self._Ad @ self._x
                   + self._Bd_act  @ F_clamped
                   + self._Bd_road @ z_r)
        return self._x.copy()

    def reset(self):
        self._x[:] = 0.0

    def state(self) -> np.ndarray:
        return self._x.copy()

    def body_Z(self)     -> float: return float(self._x[0])
    def body_theta(self) -> float: return float(self._x[1])
    def body_phi(self)   -> float: return float(self._x[2])
    def body_Zdot(self)  -> float: return float(self._x[N_DOF + 0])
    def head_accel(self) -> float:
        """Approximate head acceleration from state derivative."""
        x_dot = self._A_c @ self._x   # no input (instantaneous)
        return float(x_dot[N_DOF + 19])  # d^2 Z_h / dt^2

    def seat_accel(self) -> float:
        x_dot = self._A_c @ self._x
        return float(x_dot[N_DOF + 15])

    def body_accel(self) -> float:
        x_dot = self._A_c @ self._x
        return float(x_dot[N_DOF + 0])

    def susp_travel(self, corner: int) -> float:
        """Suspension travel at corner ci: Z_wi - Z_body_at_corner."""
        if corner < 0 or corner >= 6:
            return 0.0
        a_idx = [0, 1, 2, 0, 1, 2][corner]
        e_sign = [1, 1, 1, -1, -1, -1][corner]
        a_val  = self._p['a'][a_idx]
        hw     = self._p['W'] / 2.0
        Z_body_at_corner = (self._x[0]
                            + a_val    * self._x[1]
                            - e_sign * hw * self._x[2])
        Z_wheel = self._x[3 + corner]
        return float(Z_wheel - Z_body_at_corner)

    @property
    def Ad(self) -> np.ndarray: return self._Ad
    @property
    def Bd_act(self) -> np.ndarray: return self._Bd_act
    @property
    def params(self) -> dict: return self._p
