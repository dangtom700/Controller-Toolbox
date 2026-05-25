"""
solar_cooling_plant.py - Physics model for the Solar-Driven Cooling System
with Photovoltaic Evaporative Chimney.

Source: Ruiz, Martinez, Aguilar, Lucas
        "Analytical Modelling and Optimisation of a Solar-Driven Cooling System
         Enhanced with a Photovoltaic Evaporative Chimney"
        Applied Thermal Engineering 245 (2024) 122878

Architecture (three sequential sub-models per Fig. 3 of the paper):

  Step 1 - Evaporative area (Poppe theory, Eqs. 13-18):
      ODEs integrated via RK4 over the water temperature range [Tw_out, Tw_in].
      Outputs: Tw1 (warm), Tw2 (cool), T_ai (air leaving evaporative zone),
               Q_cond, W_comp, EER.

  Step 2 - PV convective area (Eqs. 1-12):
      Algebraic heat-balance system solved in closed form.
      Coupling input: T_ai from Step 1 (= T_i, internal air temperature).
      Outputs: Tc (cell temp), eta_PV, W_PV.

  Step 3 - Hydraulic loop (Eqs. 22-27):
      Pump operating point from system-curve / pump-curve intersection.
      Variable speed via VFD (speed ratio kr).
      Output: W_pump.

Global performance indicator (Eq. 28):
      EER_grid = Q_evap / (W_comp + W_pump - sum(W_PV))

Control input:
  - m_dot_w  : water mass-flow rate through the evaporative chimney [kg/s]
  - kr       : pump VFD speed ratio [-]

Disturbances (weather):
  - G        : solar irradiance [W/m^2]
  - T_amb    : ambient temperature [^\circC]
  - phi_amb  : ambient relative humidity [-]
  - v_w      : wind speed [m/s]
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


# ---------------------------------------------------------------------------
# Parameters
# ---------------------------------------------------------------------------
@dataclass
class PVPanelParams:
    area: float          # panel area [m^2]
    tau: float           # glass transmittance [-]
    eta_ref: float       # reference efficiency [-]
    beta_ref: float      # temperature coefficient [1/^\circC]
    T_ref: float         # reference temperature [^\circC]
    G_ref: float         # reference irradiance [W/m^2]
    T_NOCT: float
    T_ref_NOCT: float
    G_ref_NOCT: float
    gamma: float
    kg_xg: float         # k_glass / thickness_glass [W/m^2/K]
    kc_xc: float         # k_cell  / thickness_cell  [W/m^2/K]
    kt_xt: float         # k_tedlar/ thickness_tedlar [W/m^2/K]


@dataclass
class EvapAreaParams:
    me_a: float          # Merkel number correlation: Me = a * (mw/ma)^b
    me_b: float
    mair_a: float        # air mass flow: ma = a*mw^2 + b*mw + c
    mair_b: float
    mair_c: float
    vi_a: float          # internal air velocity: vi = a*mw^2 + b*mw + c
    vi_b: float
    vi_c: float
    Le: float            # Lewis factor
    cp_w: float          # specific heat of water [J/kg/K]
    cp_ma: float         # specific heat of moist air [J/kg/K]
    h_fg: float          # latent heat of vaporisation [J/kg]
    rk4_steps: int


@dataclass
class ChillerParams:
    Q_evap: float        # design cooling capacity [kW]
    T_w2_evap: float     # chiller evaporator outlet water temp [^\circC]
    a: float; b: float; c: float; d: float; e: float; f: float  # EER quadratic (Eq. 20)


@dataclass
class HydraulicsParams:
    H0: float            # nominal pump head [m]
    Q0: float            # nominal pump flow [l/s]
    eta_p0: float        # nominal pump efficiency [-]
    sys_a: float         # system curve: Hm = a + b*Q^2
    sys_b: float
    rho: float           # water density [kg/m^3]
    g: float             # gravity [m/s^2]
    kr_min: float
    kr_max: float


@dataclass
class PlantParams:
    pv: PVPanelParams
    evap: EvapAreaParams
    chiller: ChillerParams
    hydraulics: HydraulicsParams
    dt: float
    duration: float
    design_Q_m3h: float = 1.05   # design-point condenser loop flow [m^3/h]

    @classmethod
    def from_json(cls, path: str | Path) -> "PlantParams":
        with open(path, "r", encoding="utf-8") as f:
            cfg = json.load(f)

        pv_cfg = cfg["pv_panel"]
        pv = PVPanelParams(
            area=pv_cfg["area_m2"],
            tau=pv_cfg["tau"],
            eta_ref=pv_cfg["eta_ref"],
            beta_ref=pv_cfg["beta_ref"],
            T_ref=pv_cfg["T_ref_C"],
            G_ref=pv_cfg["G_ref_Wm2"],
            T_NOCT=pv_cfg["T_NOCT_C"],
            T_ref_NOCT=pv_cfg["T_ref_NOCT_C"],
            G_ref_NOCT=pv_cfg["G_ref_NOCT_Wm2"],
            gamma=pv_cfg["gamma"],
            kg_xg=pv_cfg["layers"]["k_g_over_xg"],
            kc_xc=pv_cfg["layers"]["k_c_over_xc"],
            kt_xt=pv_cfg["layers"]["k_t_over_xt"],
        )

        ev_cfg = cfg["evaporative_area"]
        evap = EvapAreaParams(
            me_a=ev_cfg["merkel_a"],
            me_b=ev_cfg["merkel_b"],
            mair_a=ev_cfg["air_flow_a"],
            mair_b=ev_cfg["air_flow_b"],
            mair_c=ev_cfg["air_flow_c"],
            vi_a=ev_cfg["vi_a"],
            vi_b=ev_cfg["vi_b"],
            vi_c=ev_cfg["vi_c"],
            Le=ev_cfg["lewis_factor"],
            cp_w=ev_cfg["cp_w_J_kgK"],
            cp_ma=ev_cfg["cp_ma_J_kgK"],
            h_fg=ev_cfg["h_fg_J_kg"],
            rk4_steps=ev_cfg["rk4_steps"],
        )

        ch_cfg = cfg["chiller"]
        eq = ch_cfg["eer_quadratic"]
        chiller = ChillerParams(
            Q_evap=ch_cfg["Q_evap_kW"],
            T_w2_evap=ch_cfg["T_w2_evap_C"],
            a=eq["a"], b=eq["b"], c=eq["c"],
            d=eq["d"], e=eq["e"], f=eq["f"],
        )

        hy_cfg = cfg["hydraulics"]
        hydraulics = HydraulicsParams(
            H0=hy_cfg["H0_m"],
            Q0=hy_cfg["Q0_ls"],
            eta_p0=hy_cfg["eta_pump0"],
            sys_a=hy_cfg["system_curve_a"],
            sys_b=hy_cfg["system_curve_b"],
            rho=hy_cfg["rho_water_kgm3"],
            g=hy_cfg["g_ms2"],
            kr_min=hy_cfg["kr_min"],
            kr_max=hy_cfg["kr_max"],
        )

        sim = cfg["simulation"]
        design_Q = cfg.get("design_point", {}).get("Q_flow_m3h", 1.05)
        return cls(pv=pv, evap=evap, chiller=chiller, hydraulics=hydraulics,
                   dt=sim["dt_s"], duration=sim["duration_s"],
                   design_Q_m3h=design_Q)


# ---------------------------------------------------------------------------
# Psychrometric helpers (standard correlations, SI units)
# ---------------------------------------------------------------------------
def _saturation_pressure_Pa(T_C: float) -> float:
    """Antoine equation: P_sat [Pa] at T [^\circC]."""
    return 610.78 * math.exp(17.269 * T_C / (237.29 + T_C))


def _humidity_ratio_sat(T_C: float, P_atm: float = 101325.0) -> float:
    """Saturation humidity ratio omega_sw [kg/kg dry air]."""
    P_sat = _saturation_pressure_Pa(T_C)
    return 0.62198 * P_sat / (P_atm - P_sat)


def _humidity_ratio(T_C: float, phi: float, P_atm: float = 101325.0) -> float:
    """Humidity ratio from dry-bulb and relative humidity."""
    P_sat = _saturation_pressure_Pa(T_C)
    return 0.62198 * phi * P_sat / (P_atm - phi * P_sat)


def _moist_air_enthalpy(T_C: float, omega: float, h_fg: float, cp_ma: float) -> float:
    """Specific enthalpy of moist air [J/kg dry air] (standard form)."""
    return cp_ma * T_C + omega * h_fg


def _sat_enthalpy(T_w: float, h_fg: float, cp_ma: float) -> float:
    """Saturated air enthalpy at water temperature Tw [^\circC]."""
    omega_sw = _humidity_ratio_sat(T_w)
    return _moist_air_enthalpy(T_w, omega_sw, h_fg, cp_ma)


# ---------------------------------------------------------------------------
# Step 1 - Evaporative area (Poppe ODEs, Eqs. 13-18)
# ---------------------------------------------------------------------------
def _poppe_rhs(Tw: float, state: list[float], evap: EvapAreaParams,
               mw_over_ma: float) -> list[float]:
    """
    RHS of Poppe ODEs: d[omega, h, Me]/dTw  (Eqs. 14-16).

    state = [omega, h_a, Me]
    """
    omega, h_a, Me = state
    omega_sw = _humidity_ratio_sat(Tw)
    h_sw = _sat_enthalpy(Tw, evap.h_fg, evap.cp_ma)
    h_w  = evap.cp_w * Tw          # enthalpy of saturated water at Tw [J/kg]

    Le = evap.Le
    d_omega = omega_sw - omega
    d_h     = h_sw - h_a

    denom = d_h + (Le - 1.0) * (d_h - d_omega * evap.h_fg) - d_omega * h_w

    if abs(denom) < 1e-12:
        return [0.0, 0.0, 0.0]

    ratio = evap.cp_w * mw_over_ma

    domega_dTw = ratio * d_omega / denom
    dh_dTw     = ratio * (1.0 + d_omega * evap.cp_w * Tw / denom)
    dMe_dTw    = evap.cp_w / denom

    return [domega_dTw, dh_dTw, dMe_dTw]


def _rk4_integrate(f, Tw_start: float, Tw_end: float,
                   y0: list[float], n: int, *args) -> list[float]:
    """Classical 4th-order Runge-Kutta integration from Tw_start to Tw_end."""
    h = (Tw_end - Tw_start) / n
    y = list(y0)
    Tw = Tw_start
    for _ in range(n):
        k1 = f(Tw,           y,               *args)
        k2 = f(Tw + 0.5*h,   [y[i] + 0.5*h*k1[i] for i in range(len(y))], *args)
        k3 = f(Tw + 0.5*h,   [y[i] + 0.5*h*k2[i] for i in range(len(y))], *args)
        k4 = f(Tw + h,       [y[i] +     h*k3[i] for i in range(len(y))], *args)
        y = [y[i] + (h/6.0)*(k1[i] + 2*k2[i] + 2*k3[i] + k4[i]) for i in range(len(y))]
        Tw += h
    return y


def solve_evaporative(m_dot_w: float, T_amb: float, phi_amb: float,
                      T_w2_chiller: float, evap: EvapAreaParams,
                      chiller: ChillerParams) -> dict:
    """
    Solve Step 1: evaporative zone + chiller.

    Inputs:
        m_dot_w     : water mass flow rate [kg/s]
        T_amb       : ambient dry-bulb temperature [^\circC]
        phi_amb     : ambient relative humidity [-]
        T_w2_chiller: chiller evaporator outlet (= evap cooler inlet) [^\circC]
        evap        : EvapAreaParams
        chiller     : ChillerParams

    Returns dict with:
        Tw1         : warm water temperature leaving evap (= chiller condenser in) [^\circC]
        Tw2         : cool water temperature entering evap (= chiller condenser out) [^\circC]
        T_ai        : air exit temperature from evap (coupling var for PV step) [^\circC]
        ma_dot      : air mass flow rate [kg/s]
        Q_cond      : condenser heat rejection [kW]
        W_comp      : compressor power [kW]
        EER         : energy efficiency ratio [-]
        Me_calc     : calculated Merkel number
        Me_target   : target Merkel number from correlation
    """
    # Air mass flow driven by water momentum (Eq. 18)
    ma = evap.mair_a * m_dot_w**2 + evap.mair_b * m_dot_w + evap.mair_c
    ma = max(ma, 1e-6)

    mw_over_ma = m_dot_w / ma

    # Merkel target from empirical correlation (Eq. 17)
    Me_target = evap.me_a * mw_over_ma ** evap.me_b

    # Chiller: EER from Eq. (20) with Tw1 as condenser inlet
    # We need Tw1. Use energy balance: Q_cond = Q_evap + W_comp
    # Iterative: guess Tw1, solve EER, check energy balance
    Q_evap = chiller.Q_evap      # [kW]
    T_w2e  = chiller.T_w2_evap   # [^\circC]
    T_w2   = T_w2_chiller         # [^\circC]  evap cool water inlet = chiller condenser outlet

    # Initial guess for warm water temp: ambient + 5 K
    Tw1 = T_amb + 5.0

    for _ in range(40):
        EER = (chiller.a
               + chiller.b * T_w2e
               + chiller.c * T_w2e**2
               + chiller.d * Tw1
               + chiller.e * Tw1**2
               + chiller.f * T_w2e * Tw1)
        EER = max(EER, 0.1)

        W_comp = Q_evap / EER                           # [kW]
        Q_cond = Q_evap + W_comp                        # [kW]

        # Condenser water energy balance: Q_cond = m_dot_w * cp_w * (Tw1 - T_w2) / 1000
        Tw1_new = T_w2 + Q_cond * 1000.0 / (m_dot_w * evap.cp_w)
        if abs(Tw1_new - Tw1) < 1e-4:
            Tw1 = Tw1_new
            break
        Tw1 = 0.6 * Tw1_new + 0.4 * Tw1   # damped update to help convergence

    # Poppe integration: parallel-flow arrangement - both water and air flow downward.
    # Water enters warm (Tw1) and exits cool (T_w2). Air enters at ambient.
    # ODEs are d[omega,h,Me]/dTw; we integrate from Tw_in=Tw1 (warm) DOWN to T_w2 (cool).
    # Because h is decreasing in this direction, dTw < 0 so the step h in RK4 is negative.
    omega_amb = _humidity_ratio(T_amb, phi_amb)
    h_amb     = _moist_air_enthalpy(T_amb, omega_amb, evap.h_fg, evap.cp_ma)

    y0 = [omega_amb, h_amb, 0.0]   # [omega_in, h_in, Me=0] at warm-water inlet

    # Integrate downward: Tw from warm (Tw1) to cool (T_w2)
    y_out = _rk4_integrate(_poppe_rhs, Tw1, T_w2, y0, evap.rk4_steps,
                            evap, mw_over_ma)

    omega_out, h_out, Me_calc = y_out

    # Recover air exit temperature from exit enthalpy and humidity ratio
    # h = cp_ma * T + omega * h_fg  =>  T = (h - omega * h_fg) / cp_ma
    T_ai = (h_out - omega_out * evap.h_fg) / evap.cp_ma

    return {
        "Tw1": Tw1,
        "Tw2": T_w2,
        "T_ai": T_ai,
        "ma_dot": ma,
        "Q_cond_kW": Q_cond,
        "W_comp_kW": W_comp,
        "EER": EER,
        "Me_calc": Me_calc,
        "Me_target": Me_target,
    }


# ---------------------------------------------------------------------------
# Step 2 - PV convective area (closed-form heat balance, Eqs. 1-8)
# ---------------------------------------------------------------------------
def solve_pv(G: float, T_amb: float, v_w: float, T_i: float,
             m_dot_w: float, pv: PVPanelParams) -> dict:
    """
    Solve the PV panel 4-layer heat balance in closed form.

    The system (Eqs. 1-4 + 8) is linear in the four unknowns [Tg, Tc, Tt, Tr]
    with eta_PV = eta_ref*(1 - beta_ref*(Tc - T_ref)) coupled nonlinearly.
    We iterate on Tc until the efficiency is self-consistent.

    Returns: Tc, Tg, Tt, Tr, eta_PV, W_PV [W].
    """
    A = pv.area
    kg_xg = pv.kg_xg
    kc_xc = pv.kc_xc
    kt_xt = pv.kt_xt

    # External convection coefficient (Eq. 5)
    he = 0.841 * v_w + 4.61

    # Internal convection coefficient (Eq. 6) via air velocity inside chimney (Eq. 7)
    vi = pv_internal_velocity(m_dot_w, pv)
    hi = 1.97 * vi + 10.0

    # Iterate on Tc (converges fast, 2-3 iterations typically)
    Tc = T_amb + 20.0   # initial guess
    for _ in range(30):
        eta = pv.eta_ref * (1.0 - pv.beta_ref * (Tc - pv.T_ref))
        eta = max(eta, 0.0)

        # Absorbed solar power per unit area minus electrical output
        q_net = G * pv.tau - eta * G   # [W/m^2] net thermal source at cell

        # Closed-form solve of Eqs. (1)-(4):
        # Eq. (1): he*(Tg - Tamb) = kg_xg*(Tc - Tg)  => Tg = (he*Tamb + kg_xg*Tc)/(he + kg_xg)
        # Eq. (3): kc_xc*(Tc - Tt) = kt_xt*(Tt - Tr) => Tt = (kc_xc*Tc + kt_xt*Tr)/(kc_xc + kt_xt)
        # Eq. (4): hi*(Tr - Ti)    = kt_xt*(Tt - Tr)  => Tr = (hi*Ti + kt_xt*Tt)/(hi + kt_xt)
        # Eq. (2): q_net*A = kg_xg*A*(Tc - Tg) + kc_xc*A*(Tc - Tt)
        # Substitute Tg(Tc), Tt(Tr(Tc)) back and solve for Tc.

        # Express Tg in terms of Tc
        # Tg = (he*T_amb + kg_xg*Tc) / (he + kg_xg)
        alpha_g = kg_xg / (he + kg_xg)        # coefficient of Tc in Tg expression
        Tg_const = he * T_amb / (he + kg_xg)  # constant part

        # Express Tr in terms of Tt, then Tt in terms of Tc via backward substitution:
        # From Eq. (4): Tr = (hi*T_i + kt_xt*Tt) / (hi + kt_xt)
        # From Eq. (3): Tt*(kc_xc + kt_xt) = kc_xc*Tc + kt_xt*Tr
        #             = kc_xc*Tc + kt_xt * [(hi*T_i + kt_xt*Tt)/(hi+kt_xt)]
        # Tt * [(kc_xc + kt_xt) - kt_xt^2/(hi+kt_xt)] = kc_xc*Tc + kt_xt*hi*T_i/(hi+kt_xt)
        denom_t = (kc_xc + kt_xt) - kt_xt**2 / (hi + kt_xt)
        alpha_t = kc_xc / denom_t             # coefficient of Tc in Tt expression
        Tt_const = (kt_xt * hi * T_i / (hi + kt_xt)) / denom_t

        # Recover Tr in terms of Tc
        # Tr = (hi*T_i + kt_xt*(alpha_t*Tc + Tt_const)) / (hi + kt_xt)
        alpha_r = kt_xt * alpha_t / (hi + kt_xt)
        Tr_const = (hi * T_i + kt_xt * Tt_const) / (hi + kt_xt)

        # Eq. (2) in terms of Tc:
        # q_net = kg_xg*(Tc - Tg) + kc_xc*(Tc - Tt)
        #       = kg_xg*(Tc - alpha_g*Tc - Tg_const) + kc_xc*(Tc - alpha_t*Tc - Tt_const)
        # q_net = kg_xg*Tc*(1 - alpha_g) - kg_xg*Tg_const
        #       + kc_xc*Tc*(1 - alpha_t) - kc_xc*Tt_const
        lhs_coeff = kg_xg * (1.0 - alpha_g) + kc_xc * (1.0 - alpha_t)
        lhs_const = -kg_xg * Tg_const - kc_xc * Tt_const

        Tc_new = (q_net - lhs_const) / lhs_coeff

        if abs(Tc_new - Tc) < 1e-5:
            Tc = Tc_new
            break
        Tc = 0.7 * Tc_new + 0.3 * Tc   # damped iteration

    Tg = alpha_g * Tc + Tg_const
    Tt = alpha_t * Tc + Tt_const
    Tr = alpha_r * Tc + Tr_const

    eta_PV = pv.eta_ref * (1.0 - pv.beta_ref * (Tc - pv.T_ref))
    eta_PV = max(eta_PV, 0.0)
    W_PV   = eta_PV * G * A    # [W]  (Eq. 12)

    return {
        "Tc_C": Tc,
        "Tg_C": Tg,
        "Tt_C": Tt,
        "Tr_C": Tr,
        "vi_ms": vi,
        "he_Wm2K": he,
        "hi_Wm2K": hi,
        "eta_PV": eta_PV,
        "W_PV_W": W_PV,
    }


def pv_internal_velocity(m_dot_w: float, pv: PVPanelParams) -> float:
    """Air velocity in convective chimney channel (Eq. 7) - stored on PVPanelParams via EvapAreaParams
    but we re-derive from the evap vi correlation reused here."""
    # The paper uses the same correlation for both areas; vi params are in evap config.
    # This function exists so the caller doesn't need to import evap params separately.
    # For the PV area solve we use T_i from Step 1 coupling instead.
    return 1.0  # placeholder; real vi injected from evap solve in the plant facade


# ---------------------------------------------------------------------------
# Step 3 - Hydraulic loop (Eqs. 22-27)
# ---------------------------------------------------------------------------
def solve_hydraulics(Q_ls: float, kr: float, hy: HydraulicsParams) -> dict:
    """
    Solve pump operating point at variable speed ratio kr.

    Pump head curve (Eq. 26):  Hm = H0*kr^2 * [1 - (Q/(kr*Q0))^2]
    System curve  (Eq. 22):    Hs = a + b*Q^2
    Operating point:           Hm(Q) = Hs(Q)  => solve for Q, then W_pump.

    We solve the intersection analytically.
    H0*kr^2 - H0*(Q/Q0)^2 = sys_a + sys_b*Q^2
    H0*kr^2 - sys_a = Q^2 * (H0/Q0^2 + sys_b)
    Q_op = sqrt((H0*kr^2 - sys_a) / (H0/Q0^2 + sys_b))   [l/s]

    Pump efficiency at variable speed (Eq. 27):
    eta_p = eta_p0 * (Q/(kr*Q0)) * (1 - Q/(kr*Q0))
    """
    kr = float(max(hy.kr_min, min(hy.kr_max, kr)))

    numerator = hy.H0 * kr**2 - hy.sys_a
    denominator = hy.H0 / hy.Q0**2 + hy.sys_b

    if numerator <= 0.0 or denominator <= 0.0:
        return {"Q_op_ls": 0.0, "Hm_m": hy.sys_a, "W_pump_W": 0.0, "eta_pump": 0.0, "kr": kr}

    Q_op = math.sqrt(numerator / denominator)   # [l/s]
    Q_op_m3s = Q_op * 1e-3                       # [m^3/s]

    Hm = hy.sys_a + hy.sys_b * Q_op**2          # [m]

    ratio = Q_op / (kr * hy.Q0)
    ratio = min(ratio, 1.0)
    eta_p = hy.eta_p0 * ratio * (1.0 - ratio)
    eta_p = max(eta_p, 1e-6)

    W_pump = hy.rho * hy.g * Q_op_m3s * Hm / eta_p   # [W]

    return {
        "Q_op_ls": Q_op,
        "Hm_m": Hm,
        "W_pump_W": W_pump,
        "eta_pump": eta_p,
        "kr": kr,
    }


# ---------------------------------------------------------------------------
# Global performance (Eq. 28)
# ---------------------------------------------------------------------------
def eer_grid(Q_evap_kW: float, W_comp_kW: float, W_pump_W: float, W_PV_W: float) -> float:
    """EER_grid = Q_evap / (W_comp + W_pump - sum W_PV)  [kW input basis]."""
    W_net_kW = W_comp_kW + W_pump_W / 1000.0 - W_PV_W / 1000.0
    if W_net_kW <= 0.0:
        return float("inf")
    return Q_evap_kW / W_net_kW


# ---------------------------------------------------------------------------
# Plant state & facade
# ---------------------------------------------------------------------------
@dataclass
class WeatherInput:
    G: float        # solar irradiance [W/m^2]
    T_amb: float    # ambient temperature [^\circC]
    phi_amb: float  # relative humidity [-]
    v_w: float      # wind speed [m/s]


@dataclass
class ControlInput:
    m_dot_w: float  # water mass flow rate [kg/s]
    kr: float       # pump VFD speed ratio [-]


@dataclass
class PlantOutput:
    # Evaporative zone
    Tw1_C: float
    Tw2_C: float
    T_ai_C: float
    ma_dot_kgs: float
    Q_cond_kW: float
    W_comp_kW: float
    EER: float
    Me_calc: float
    Me_target: float
    # PV zone
    Tc_C: float
    eta_PV: float
    W_PV_W: float
    vi_ms: float
    # Hydraulics
    Q_op_ls: float
    Hm_m: float
    W_pump_W: float
    eta_pump: float
    # Global
    EER_grid: float
    W_net_kW: float


class SolarCoolingPlant:
    """
    Steady-state algebraic plant for the Solar-Driven Cooling System.

    The plant has no internal ODE state - all three sub-models are algebraic
    or integrate over the spatial coordinate Tw (not time). Each call to
    step() returns the instantaneous operating point given the current
    disturbances and control inputs.

    Control inputs:
        m_dot_w  : water mass-flow rate [kg/s] - governs evaporation, Vi, EER
        kr       : pump speed ratio [-]          - governs condenser loop flow

    Disturbances:
        G        : solar irradiance [W/m^2]
        T_amb    : ambient dry-bulb temperature [^\circC]
        phi_amb  : ambient relative humidity [-]
        v_w      : wind speed [m/s]
    """

    def __init__(self, params: PlantParams):
        self.p = params
        self._last_output: Optional[PlantOutput] = None

    @classmethod
    def from_json(cls, path: str | Path) -> "SolarCoolingPlant":
        return cls(PlantParams.from_json(path))

    def step(self, weather: WeatherInput, control: ControlInput) -> PlantOutput:
        """Compute one steady-state operating point."""
        p = self.p
        u = control
        w = weather

        # Clamp to validity range of empirical Eq. 7 and 18 (Lucas et al.)
        m_dot_w = max(0.02, min(0.22, u.m_dot_w))
        kr = max(p.hydraulics.kr_min, min(p.hydraulics.kr_max, u.kr))

        # Step 1: Evaporative zone + chiller
        evap_out = solve_evaporative(
            m_dot_w=m_dot_w,
            T_amb=w.T_amb,
            phi_amb=w.phi_amb,
            T_w2_chiller=p.chiller.T_w2_evap,
            evap=p.evap,
            chiller=p.chiller,
        )

        # Step 2: PV convective area (T_i = T_ai from Step 1)
        # vi from evap correlation (Eq. 7) - same correlation used for the convective area
        vi = (p.evap.vi_a * m_dot_w**2
              + p.evap.vi_b * m_dot_w
              + p.evap.vi_c)
        vi = max(vi, 0.0)

        he = 0.841 * w.v_w + 4.61
        hi = 1.97 * vi + 10.0

        T_i = evap_out["T_ai"]
        G = w.G

        pv_out = _solve_pv_internal(G, w.T_amb, he, hi, T_i, p.pv)
        pv_out["vi_ms"] = vi

        # Step 3: Hydraulic loop - flow in l/s from design m3/h
        Q_ls = p.design_Q_m3h / 3.6
        hyd_out = solve_hydraulics(Q_ls=Q_ls, kr=kr, hy=p.hydraulics)

        # Global EER_grid
        W_net_kW = (evap_out["W_comp_kW"]
                    + hyd_out["W_pump_W"] / 1000.0
                    - pv_out["W_PV_W"] / 1000.0)
        EER_g = eer_grid(p.chiller.Q_evap, evap_out["W_comp_kW"],
                         hyd_out["W_pump_W"], pv_out["W_PV_W"])

        out = PlantOutput(
            Tw1_C=evap_out["Tw1"],
            Tw2_C=evap_out["Tw2"],
            T_ai_C=evap_out["T_ai"],
            ma_dot_kgs=evap_out["ma_dot"],
            Q_cond_kW=evap_out["Q_cond_kW"],
            W_comp_kW=evap_out["W_comp_kW"],
            EER=evap_out["EER"],
            Me_calc=evap_out["Me_calc"],
            Me_target=evap_out["Me_target"],
            Tc_C=pv_out["Tc_C"],
            eta_PV=pv_out["eta_PV"],
            W_PV_W=pv_out["W_PV_W"],
            vi_ms=vi,
            Q_op_ls=hyd_out["Q_op_ls"],
            Hm_m=hyd_out["Hm_m"],
            W_pump_W=hyd_out["W_pump_W"],
            eta_pump=hyd_out["eta_pump"],
            EER_grid=EER_g,
            W_net_kW=W_net_kW,
        )
        self._last_output = out
        return out

    @property
    def last_output(self) -> Optional[PlantOutput]:
        return self._last_output


def _solve_pv_internal(G: float, T_amb: float, he: float, hi: float,
                       T_i: float, pv: PVPanelParams) -> dict:
    """Internal PV heat-balance solver (closed-form + Tc iteration)."""
    A = pv.area
    kg_xg = pv.kg_xg
    kc_xc = pv.kc_xc
    kt_xt = pv.kt_xt

    Tc = T_amb + 20.0
    for _ in range(30):
        eta = pv.eta_ref * (1.0 - pv.beta_ref * (Tc - pv.T_ref))
        eta = max(eta, 0.0)

        q_net = G * pv.tau - eta * G

        alpha_g = kg_xg / (he + kg_xg)
        Tg_const = he * T_amb / (he + kg_xg)

        denom_t = (kc_xc + kt_xt) - kt_xt**2 / (hi + kt_xt)
        if abs(denom_t) < 1e-12:
            break
        alpha_t = kc_xc / denom_t
        Tt_const = (kt_xt * hi * T_i / (hi + kt_xt)) / denom_t

        alpha_r = kt_xt * alpha_t / (hi + kt_xt)
        Tr_const = (hi * T_i + kt_xt * Tt_const) / (hi + kt_xt)

        lhs_coeff = kg_xg * (1.0 - alpha_g) + kc_xc * (1.0 - alpha_t)
        if abs(lhs_coeff) < 1e-12:
            break
        lhs_const = -kg_xg * Tg_const - kc_xc * Tt_const

        Tc_new = (q_net - lhs_const) / lhs_coeff
        if abs(Tc_new - Tc) < 1e-5:
            Tc = Tc_new
            break
        Tc = 0.7 * Tc_new + 0.3 * Tc

    Tg = kg_xg / (he + kg_xg) * Tc + he * T_amb / (he + kg_xg)
    denom_t = (kc_xc + kt_xt) - kt_xt**2 / (hi + kt_xt)
    alpha_t = kc_xc / denom_t if abs(denom_t) > 1e-12 else 0.0
    Tt_const = (kt_xt * hi * T_i / (hi + kt_xt)) / denom_t if abs(denom_t) > 1e-12 else T_i
    Tt = alpha_t * Tc + Tt_const
    Tr = (hi * T_i + kt_xt * Tt) / (hi + kt_xt)

    eta_PV = pv.eta_ref * (1.0 - pv.beta_ref * (Tc - pv.T_ref))
    eta_PV = max(eta_PV, 0.0)
    W_PV   = eta_PV * G * A

    return {
        "Tc_C": Tc,
        "Tg_C": Tg,
        "Tt_C": Tt,
        "Tr_C": Tr,
        "eta_PV": eta_PV,
        "W_PV_W": W_PV,
    }


# ---------------------------------------------------------------------------
# Smoke tests
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    cfg_path = Path(__file__).resolve().parent.parent / "config" / "plant_params.json"
    params   = PlantParams.from_json(cfg_path)

    # Load design point from JSON for cross-check
    with open(cfg_path, "r") as f:
        raw = json.load(f)
    dp_cfg = raw["design_point"]
    weather = WeatherInput(
        G=dp_cfg["G_Wm2"],
        T_amb=dp_cfg["T_amb_C"],
        phi_amb=dp_cfg["phi_amb"],
        v_w=dp_cfg["v_w_ms"],
    )
    # m_dot_w here is the chimney spray water flow (~0.05-0.20 kg/s per Eq. 7 validity range)
    control = ControlInput(m_dot_w=0.10, kr=0.85)

    plant = SolarCoolingPlant(params)
    out   = plant.step(weather, control)

    print("=== Design-point smoke test ===")
    print(f"  EER          = {out.EER:.3f}     (paper: ~3-5 typical)")
    print(f"  EER_grid     = {out.EER_grid:.3f}")
    print(f"  Tw1 (warm)   = {out.Tw1_C:.2f} ^\circC")
    print(f"  Tw2 (cool)   = {out.Tw2_C:.2f} ^\circC  (target: {params.chiller.T_w2_evap} ^\circC)")
    print(f"  T_ai         = {out.T_ai_C:.2f} ^\circC  (ambient: {weather.T_amb} ^\circC)")
    print(f"  Tc (cell)    = {out.Tc_C:.2f} ^\circC")
    print(f"  eta_PV       = {out.eta_PV*100:.2f} %   (ref: {params.pv.eta_ref*100:.1f} %)")
    print(f"  W_PV         = {out.W_PV_W:.1f} W")
    print(f"  W_comp       = {out.W_comp_kW*1000:.1f} W")
    print(f"  W_pump       = {out.W_pump_W:.1f} W")
    print(f"  Me_calc      = {out.Me_calc:.4f}  Me_target = {out.Me_target:.4f}")
    print(f"  vi           = {out.vi_ms:.3f} m/s")

    # Assertion: EER within physical range
    assert 0.5 < out.EER < 20.0, f"EER out of range: {out.EER}"
    assert out.Tc_C > weather.T_amb, "Cell must be hotter than ambient"
    assert out.W_PV_W > 0.0, "PV must produce positive power"
    assert out.W_pump_W > 0.0, "Pump must consume positive power"
    print("PASS: all assertions satisfied.")
