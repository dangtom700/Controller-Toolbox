#include "solar_plant.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>
#include <algorithm>
#include <array>
#include <cmath>

using json = nlohmann::json;

namespace solar {

// ---------------------------------------------------------------------------
// JSON loader
// ---------------------------------------------------------------------------
PlantParams PlantParams::fromJson(const std::string& path)
{
    std::ifstream f(path);
    if (!f.is_open())
        throw std::runtime_error("SolarPlant: cannot open config: " + path);
    json cfg = json::parse(f);

    PlantParams p{};

    // PV panel
    auto& pj = cfg["pv_panel"];
    p.pv.area        = pj["area_m2"];
    p.pv.tau         = pj["tau"];
    p.pv.eta_ref     = pj["eta_ref"];
    p.pv.beta_ref    = pj["beta_ref"];
    p.pv.T_ref       = pj["T_ref_C"];
    p.pv.G_ref       = pj["G_ref_Wm2"];
    p.pv.T_NOCT      = pj["T_NOCT_C"];
    p.pv.T_ref_NOCT  = pj["T_ref_NOCT_C"];
    p.pv.G_ref_NOCT  = pj["G_ref_NOCT_Wm2"];
    p.pv.gamma       = pj["gamma"];
    p.pv.kg_xg       = pj["layers"]["k_g_over_xg"];
    p.pv.kc_xc       = pj["layers"]["k_c_over_xc"];
    p.pv.kt_xt       = pj["layers"]["k_t_over_xt"];

    // Evaporative area
    auto& ej = cfg["evaporative_area"];
    p.evap.me_a      = ej["merkel_a"];
    p.evap.me_b      = ej["merkel_b"];
    p.evap.mair_a    = ej["air_flow_a"];
    p.evap.mair_b    = ej["air_flow_b"];
    p.evap.mair_c    = ej["air_flow_c"];
    p.evap.vi_a      = ej["vi_a"];
    p.evap.vi_b      = ej["vi_b"];
    p.evap.vi_c      = ej["vi_c"];
    p.evap.Le        = ej["lewis_factor"];
    p.evap.cp_w      = ej["cp_w_J_kgK"];
    p.evap.cp_ma     = ej["cp_ma_J_kgK"];
    p.evap.h_fg      = ej["h_fg_J_kg"];
    p.evap.rk4_steps = ej["rk4_steps"];

    // Chiller
    auto& ch = cfg["chiller"];
    p.chiller.Q_evap_kW = ch["Q_evap_kW"];
    p.chiller.T_w2_evap = ch["T_w2_evap_C"];
    auto& eq = ch["eer_quadratic"];
    p.chiller.a = eq["a"];  p.chiller.b = eq["b"];  p.chiller.c = eq["c"];
    p.chiller.d = eq["d"];  p.chiller.e = eq["e"];  p.chiller.f = eq["f"];

    // Hydraulics
    auto& hj = cfg["hydraulics"];
    p.hydraulics.H0      = hj["H0_m"];
    p.hydraulics.Q0      = hj["Q0_ls"];
    p.hydraulics.eta_p0  = hj["eta_pump0"];
    p.hydraulics.sys_a   = hj["system_curve_a"];
    p.hydraulics.sys_b   = hj["system_curve_b"];
    p.hydraulics.rho     = hj["rho_water_kgm3"];
    p.hydraulics.g       = hj["g_ms2"];
    p.hydraulics.kr_min  = hj["kr_min"];
    p.hydraulics.kr_max  = hj["kr_max"];

    // Simulation
    p.dt       = cfg["simulation"]["dt_s"];
    p.duration = cfg["simulation"]["duration_s"];
    p.design_Q_m3h = cfg.value("design_point", json{}).value("Q_flow_m3h", 1.05);

    return p;
}

// ---------------------------------------------------------------------------
// Psychrometric helpers
// ---------------------------------------------------------------------------
static double satPressure(double T_C)
{
    // Antoine equation: P_sat [Pa]
    return 610.78 * std::exp(17.269 * T_C / (237.29 + T_C));
}

static double humidityRatioSat(double T_C, double P_atm = 101325.0)
{
    double Ps = satPressure(T_C);
    return 0.62198 * Ps / (P_atm - Ps);
}

static double humidityRatio(double T_C, double phi, double P_atm = 101325.0)
{
    double Ps = satPressure(T_C);
    return 0.62198 * phi * Ps / (P_atm - phi * Ps);
}

static double moistAirEnthalpy(double T_C, double omega, double h_fg, double cp_ma)
{
    return cp_ma * T_C + omega * h_fg;
}

static double satEnthalpy(double T_w, double h_fg, double cp_ma)
{
    double omega_sw = humidityRatioSat(T_w);
    return moistAirEnthalpy(T_w, omega_sw, h_fg, cp_ma);
}

// ---------------------------------------------------------------------------
// Step 1a - Poppe ODEs RHS  (Eqs. 14-16)
// state = {omega, h_a, Me}
// Returns d(state)/dTw
// ---------------------------------------------------------------------------
using State3 = std::array<double, 3>;

static State3 poppeRhs(double Tw, const State3& s,
                       const EvapParams& ev, double mw_over_ma)
{
    double omega   = s[0];
    double h_a     = s[1];
    // s[2] = Me (accumulates)

    double omega_sw = humidityRatioSat(Tw);
    double h_sw     = satEnthalpy(Tw, ev.h_fg, ev.cp_ma);
    double h_w      = ev.cp_w * Tw;

    double d_omega = omega_sw - omega;
    double d_h     = h_sw    - h_a;

    double denom = d_h + (ev.Le - 1.0) * (d_h - d_omega * ev.h_fg)
                       - d_omega * h_w;

    if (std::abs(denom) < 1e-12)
        return {0.0, 0.0, 0.0};

    double ratio = ev.cp_w * mw_over_ma;

    State3 ds{};
    ds[0] = ratio * d_omega / denom;                                      // domega/dTw
    ds[1] = ratio * (1.0 + d_omega * ev.cp_w * Tw / denom);              // dh/dTw
    ds[2] = ev.cp_w / denom;                                              // dMe/dTw
    return ds;
}

static State3 rk4Step(double Tw, double h,
                      const State3& y,
                      const EvapParams& ev, double mw_over_ma)
{
    auto k1 = poppeRhs(Tw,           y,                             ev, mw_over_ma);
    State3 y2; for (int i = 0; i < 3; ++i) y2[i] = y[i] + 0.5*h*k1[i];
    auto k2 = poppeRhs(Tw + 0.5*h,   y2,                            ev, mw_over_ma);
    State3 y3; for (int i = 0; i < 3; ++i) y3[i] = y[i] + 0.5*h*k2[i];
    auto k3 = poppeRhs(Tw + 0.5*h,   y3,                            ev, mw_over_ma);
    State3 y4; for (int i = 0; i < 3; ++i) y4[i] = y[i] +     h*k3[i];
    auto k4 = poppeRhs(Tw + h,        y4,                            ev, mw_over_ma);

    State3 out;
    for (int i = 0; i < 3; ++i)
        out[i] = y[i] + (h / 6.0) * (k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]);
    return out;
}

// ---------------------------------------------------------------------------
// Step 1b - Evaporative zone + chiller energy balance
// ---------------------------------------------------------------------------
struct EvapResult {
    double Tw1;         // warm water temp  [^\circC]
    double T_ai;        // air exit temp    [^\circC]
    double ma_dot;      // air mass flow    [kg/s]
    double Q_cond_kW;
    double W_comp_kW;
    double EER;
    double Me_calc;
    double Me_target;
};

static EvapResult solveEvaporative(double m_dot_w,
                                   double T_amb, double phi_amb,
                                   const EvapParams& ev,
                                   const ChillerParams& ch)
{
    // Air mass flow (Eq. 18)
    double ma = ev.mair_a * m_dot_w*m_dot_w + ev.mair_b * m_dot_w + ev.mair_c;
    ma = std::max(ma, 1e-6);

    double mw_over_ma = m_dot_w / ma;

    // Merkel number target (Eq. 17)
    double Me_target = ev.me_a * std::pow(mw_over_ma, ev.me_b);

    // Chiller iterative energy balance
    double Q_evap = ch.Q_evap_kW;
    double T_w2e  = ch.T_w2_evap;
    double T_w2   = T_w2e;          // condenser cool-water outlet = chiller evap inlet temp

    double Tw1 = T_amb + 5.0;       // initial guess
    double EER = 3.0, W_comp = Q_evap / EER, Q_cond = Q_evap + W_comp;

    for (int iter = 0; iter < 60; ++iter) {
        EER = ch.a
            + ch.b * T_w2e
            + ch.c * T_w2e * T_w2e
            + ch.d * Tw1
            + ch.e * Tw1 * Tw1
            + ch.f * T_w2e * Tw1;
        EER    = std::max(EER, 0.1);
        W_comp = Q_evap / EER;
        Q_cond = Q_evap + W_comp;

        // Condenser water energy balance:  Q_cond = m_dot_w * cp_w * (Tw1 - T_w2) / 1000
        double Tw1_new = T_w2 + Q_cond * 1000.0 / (m_dot_w * ev.cp_w);
        if (std::abs(Tw1_new - Tw1) < 1e-5) { Tw1 = Tw1_new; break; }
        Tw1 = 0.6 * Tw1_new + 0.4 * Tw1;
    }

    // Poppe integration - parallel-flow, integrate from Tw1 (warm) down to T_w2 (cool)
    double omega0 = humidityRatio(T_amb, phi_amb);
    double h0     = moistAirEnthalpy(T_amb, omega0, ev.h_fg, ev.cp_ma);
    State3 y      = {omega0, h0, 0.0};

    double dTw = (T_w2 - Tw1) / ev.rk4_steps;   // negative step (descending)
    double Tw  = Tw1;
    for (int i = 0; i < ev.rk4_steps; ++i) {
        y  = rk4Step(Tw, dTw, y, ev, mw_over_ma);
        Tw += dTw;
    }

    double omega_out = y[0];
    double h_out     = y[1];
    double Me_calc   = y[2];

    // Air exit temperature from exit enthalpy
    double T_ai = (h_out - omega_out * ev.h_fg) / ev.cp_ma;

    return {Tw1, T_ai, ma, Q_cond, W_comp, EER, Me_calc, Me_target};
}

// ---------------------------------------------------------------------------
// Step 2 - PV convective area  (Eqs. 1-8, iterate on Tc)
// ---------------------------------------------------------------------------
struct PVResult {
    double Tc;
    double eta_PV;
    double W_PV;
};

static PVResult solvePV(double G, double T_amb,
                        double he, double hi, double T_i,
                        const PVParams& pv)
{
    double A   = pv.area;
    double Tc  = T_amb + 20.0;   // initial guess

    for (int iter = 0; iter < 40; ++iter) {
        double eta = pv.eta_ref * (1.0 - pv.beta_ref * (Tc - pv.T_ref));
        eta = std::max(eta, 0.0);

        double q_net = G * pv.tau - eta * G;   // net thermal source at cell [W/m^2]

        // Express Tg, Tt, Tr linearly in Tc (derived from Eqs. 1-4)
        double alpha_g  = pv.kg_xg / (he + pv.kg_xg);
        double Tg_const = he * T_amb / (he + pv.kg_xg);

        double denom_t  = (pv.kc_xc + pv.kt_xt) - pv.kt_xt * pv.kt_xt / (hi + pv.kt_xt);
        if (std::abs(denom_t) < 1e-12) break;

        double alpha_t  = pv.kc_xc / denom_t;
        double Tt_const = (pv.kt_xt * hi * T_i / (hi + pv.kt_xt)) / denom_t;

        // Eq. 2 (energy balance on cell) linearised in Tc:
        double lhs_coeff = pv.kg_xg * (1.0 - alpha_g) + pv.kc_xc * (1.0 - alpha_t);
        if (std::abs(lhs_coeff) < 1e-12) break;

        double lhs_const = -pv.kg_xg * Tg_const - pv.kc_xc * Tt_const;
        double Tc_new    = (q_net - lhs_const) / lhs_coeff;

        if (std::abs(Tc_new - Tc) < 1e-5) { Tc = Tc_new; break; }
        Tc = 0.7 * Tc_new + 0.3 * Tc;
    }

    double eta_PV = pv.eta_ref * (1.0 - pv.beta_ref * (Tc - pv.T_ref));
    eta_PV = std::max(eta_PV, 0.0);
    double W_PV   = eta_PV * G * A;

    return {Tc, eta_PV, W_PV};
}

// ---------------------------------------------------------------------------
// Step 3 - Hydraulic loop  (Eqs. 22-27)
// ---------------------------------------------------------------------------
struct HydResult {
    double Q_op;        // [l/s]
    double Hm;          // [m]
    double W_pump;      // [W]
    double eta_pump;
};

static HydResult solveHydraulics(double kr, const HydraulicsParams& hy)
{
    kr = std::clamp(kr, hy.kr_min, hy.kr_max);

    // Analytical pump/system intersection (Eq. 26 == Eq. 22)
    double num = hy.H0 * kr * kr - hy.sys_a;
    double den = hy.H0 / (hy.Q0 * hy.Q0) + hy.sys_b;

    if (num <= 0.0 || den <= 0.0)
        return {0.0, hy.sys_a, 0.0, 0.0};

    double Q_op    = std::sqrt(num / den);          // [l/s]
    double Q_op_m3 = Q_op * 1e-3;                  // [m^3/s]
    double Hm      = hy.sys_a + hy.sys_b * Q_op * Q_op;

    double ratio   = Q_op / (kr * hy.Q0);
    ratio          = std::min(ratio, 1.0);
    double eta_p   = std::max(hy.eta_p0 * ratio * (1.0 - ratio), 1e-6);
    double W_pump  = hy.rho * hy.g * Q_op_m3 * Hm / eta_p;

    return {Q_op, Hm, W_pump, eta_p};
}

// ---------------------------------------------------------------------------
// SolarCoolingPlant
// ---------------------------------------------------------------------------
SolarCoolingPlant::SolarCoolingPlant(const PlantParams& params)
    : p_(params)
{}

SolarCoolingPlant SolarCoolingPlant::fromJson(const std::string& path)
{
    return SolarCoolingPlant(PlantParams::fromJson(path));
}

PlantOutput SolarCoolingPlant::step(const WeatherInput& w, const ControlInput& u)
{
    const auto& ev = p_.evap;
    const auto& pv = p_.pv;
    const auto& hy = p_.hydraulics;

    // Clamp control inputs to validity range of empirical correlations
    double m_dot_w = std::clamp(u.m_dot_w, 0.02, 0.22);
    double kr      = std::clamp(u.kr, hy.kr_min, hy.kr_max);

    // Step 1: evaporative zone + chiller
    EvapResult er = solveEvaporative(m_dot_w, w.T_amb, w.phi_amb, ev, p_.chiller);

    // Step 2: PV area
    // Internal air velocity (Eq. 7)
    double vi = ev.vi_a * m_dot_w * m_dot_w + ev.vi_b * m_dot_w + ev.vi_c;
    vi = std::max(vi, 0.0);

    double he = 0.841 * w.v_w + 4.61;   // Eq. 5
    double hi = 1.97  * vi   + 10.0;    // Eq. 6

    PVResult pvr = solvePV(w.G, w.T_amb, he, hi, er.T_ai, pv);

    // Step 3: hydraulic loop
    HydResult hr = solveHydraulics(kr, hy);

    // Global EER_grid (Eq. 28)
    double W_net_kW = er.W_comp_kW + hr.W_pump / 1000.0 - pvr.W_PV / 1000.0;
    double EER_grid = (W_net_kW > 0.0) ? p_.chiller.Q_evap_kW / W_net_kW
                                        : 1e9;

    PlantOutput out{};
    out.Tw1_C      = er.Tw1;
    out.Tw2_C      = p_.chiller.T_w2_evap;
    out.T_ai_C     = er.T_ai;
    out.ma_dot     = er.ma_dot;
    out.Q_cond_kW  = er.Q_cond_kW;
    out.W_comp_kW  = er.W_comp_kW;
    out.EER        = er.EER;
    out.Me_calc    = er.Me_calc;
    out.Me_target  = er.Me_target;
    out.Tc_C       = pvr.Tc;
    out.eta_PV     = pvr.eta_PV;
    out.W_PV_W     = pvr.W_PV;
    out.vi_ms      = vi;
    out.Q_op_ls    = hr.Q_op;
    out.Hm_m       = hr.Hm;
    out.W_pump_W   = hr.W_pump;
    out.eta_pump   = hr.eta_pump;
    out.EER_grid   = EER_grid;
    out.W_net_kW   = W_net_kW;

    last_ = out;
    return out;
}

} // namespace solar
