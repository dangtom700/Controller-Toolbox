#include "sotec_plant.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <cmath>
#include <stdexcept>
#include <algorithm>

namespace sotec {

// ---------------------------------------------------------------------------
// PlantParams JSON loading
// ---------------------------------------------------------------------------
PlantParams PlantParams::fromJson(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) throw std::runtime_error("Cannot open " + path);
    nlohmann::json j;
    f >> j;
    PlantParams p;
    auto get = [&](const char* key, double def) -> double {
        return j.contains(key) ? j[key].get<double>() : def;
    };
    p.A_coll      = get("A_coll", p.A_coll);
    p.eta_coll    = get("eta_coll", p.eta_coll);
    p.U_coll      = get("U_coll", p.U_coll);
    p.m_coll      = get("m_coll", p.m_coll);
    p.cp_f        = get("cp_f", p.cp_f);
    p.m_tank      = get("m_tank", p.m_tank);
    p.cp_wf       = get("cp_wf", p.cp_wf);
    p.eta_exp     = get("eta_exp", p.eta_exp);
    p.eta_pump    = get("eta_pump", p.eta_pump);
    p.rho_wf      = get("rho_wf", p.rho_wf);
    p.P_cond      = get("P_cond", p.P_cond);
    p.a0          = get("a0", p.a0);
    p.a1          = get("a1", p.a1);
    p.a2          = get("a2", p.a2);
    p.P_inlet_max = get("P_inlet_max", p.P_inlet_max);
    p.dp_f        = get("dp_f", p.dp_f);
    p.rho_f       = get("rho_f", p.rho_f);
    p.eta_f       = get("eta_f", p.eta_f);
    p.m_dot_f_min  = get("m_dot_f_min",  p.m_dot_f_min);
    p.m_dot_f_max  = get("m_dot_f_max",  p.m_dot_f_max);
    p.m_dot_wf_min = get("m_dot_wf_min", p.m_dot_wf_min);
    p.m_dot_wf_max = get("m_dot_wf_max", p.m_dot_wf_max);
    p.T_c_nom     = get("T_c_nom", p.T_c_nom);
    p.Ts          = get("Ts", p.Ts);
    p.duration    = get("duration", p.duration);
    return p;
}

// ---------------------------------------------------------------------------
// ScenarioConfig
// ---------------------------------------------------------------------------
double ScenarioConfig::GAt(double t) const {
    if (g_type == "step") {
        return (t >= t_G_step && t_G_step >= 0) ? G_1 : G_0;
    } else if (g_type == "sinusoidal") {
        return G_0 + G_amp * std::sin(G_freq * t);
    }
    return G_0; // constant
}

double ScenarioConfig::T_h_refAt(double t) const {
    if (T_h_ref_type == "step" && t >= t_T_h_step && t_T_h_step >= 0)
        return T_h_ref_1;
    return T_h_ref;
}

Disturbance ScenarioConfig::distAt(double t) const {
    Disturbance d;
    d.G     = GAt(t);
    d.T_amb = T_amb_0;
    d.T_c   = T_c_0;
    return d;
}

ScenarioConfig ScenarioConfig::fromJson(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) throw std::runtime_error("Cannot open " + path);
    nlohmann::json j;
    f >> j;
    ScenarioConfig s;
    auto getd = [&](const char* key, double def) -> double {
        return j.contains(key) ? j[key].get<double>() : def;
    };
    auto gets = [&](const char* key, std::string def) -> std::string {
        return j.contains(key) ? j[key].get<std::string>() : def;
    };
    s.id          = gets("id", "");
    s.description = gets("description", "");
    s.T_h_ref     = getd("T_h_ref", s.T_h_ref);
    s.T_h_init    = getd("T_h_init", s.T_h_init);
    s.T_coll_init = getd("T_coll_init", s.T_coll_init);
    s.duration_s  = getd("duration_s", s.duration_s);
    s.g_type      = gets("g_type", s.g_type);
    s.G_0         = getd("G_0", s.G_0);
    s.G_1         = getd("G_1", s.G_1);
    s.t_G_step    = getd("t_G_step", s.t_G_step);
    s.G_amp       = getd("G_amp", s.G_amp);
    s.G_freq      = getd("G_freq", s.G_freq);
    s.T_h_ref_type = gets("T_h_ref_type", s.T_h_ref_type);
    s.T_h_ref_1   = getd("T_h_ref_1", s.T_h_ref_1);
    s.t_T_h_step  = getd("t_T_h_step", s.t_T_h_step);
    s.T_amb_0     = getd("T_amb_0", s.T_amb_0);
    s.T_c_0       = getd("T_c_0", s.T_c_0);
    return s;
}

// ---------------------------------------------------------------------------
// SotecPlant
// ---------------------------------------------------------------------------
SotecPlant::SotecPlant(const PlantParams& p) : p_(p) {
    x_ << 20.0, 20.0;
}

void SotecPlant::reset(double T_h_init, double T_coll_init) {
    x_(0) = T_h_init;
    x_(1) = T_coll_init;
}

double SotecPlant::m_dot_wf_max_safe(double T_h) const {
    const double lim = (p_.P_inlet_max - p_.a0 - p_.a1 * T_h) / p_.a2;
    return std::clamp(lim, p_.m_dot_wf_min, p_.m_dot_wf_max);
}

double SotecPlant::Q_evap_from_state(double T_h, double m_dot_wf, double T_c) const {
    // Heat extracted by the ORC evaporator from the hot tank
    // Q_evap = m_dot_wf * cp_wf * (T_h - T_c)  [simplified pinch model]
    return m_dot_wf * p_.cp_wf * std::max(0.0, T_h - T_c);
}

OrcState SotecPlant::computeOrc(double m_dot_f, double m_dot_wf, const Disturbance& d) const {
    OrcState orc;
    const double T_h = x_(0);
    const double T_c = d.T_c;

    // Clamp m_dot_wf to pressure limit
    const double mwf_safe  = m_dot_wf_max_safe(T_h);
    const double m_dot_wf_ = std::clamp(m_dot_wf, p_.m_dot_wf_min, mwf_safe);

    // Expander inlet pressure [MPa]
    orc.P_inlet = p_.a0 + p_.a1 * T_h + p_.a2 * m_dot_wf_;

    // Superheating degree [^\circC] - approximated as T_h minus saturation temp
    // Sat. temp of R134a around 1 MPa approx = 39.4^\circC; linear approx
    const double T_sat = 20.0 + 20.0 * (orc.P_inlet - 0.5);  // rough estimate
    orc.delta_T_super = std::max(0.0, T_h - T_sat);

    // Evaporator heat input
    orc.Q_evap = Q_evap_from_state(T_h, m_dot_wf_, T_c);

    // Expander: dh_exp = eta_exp * cp_wf * delta_T_super (simplified)
    orc.dh_exp = p_.eta_exp * p_.cp_wf * std::max(0.0, T_h - T_c) * 0.2;

    // Gross power
    orc.W_gross = m_dot_wf_ * orc.dh_exp;

    // WF pump power
    const double dP_wf = (orc.P_inlet - p_.P_cond) * 1e6; // Pa
    orc.W_pump_wf = m_dot_wf_ * dP_wf / (p_.rho_wf * p_.eta_pump);

    // Solar collector pump power
    orc.W_solar_pump = m_dot_f * p_.dp_f / (p_.rho_f * p_.eta_f);

    // Net power
    orc.W_net = orc.W_gross - orc.W_pump_wf - orc.W_solar_pump;

    // Thermal efficiency
    orc.eta_th = (orc.Q_evap > 1.0) ? orc.W_net / orc.Q_evap : 0.0;

    return orc;
}

void SotecPlant::step(double m_dot_f, double m_dot_wf, const Disturbance& d) {
    const double T_h    = x_(0);
    const double T_coll = x_(1);
    const double G      = d.G;
    const double T_amb  = d.T_amb;
    const double T_c    = d.T_c;

    // Clamp m_dot_wf to pressure safety
    const double mwf_safe = m_dot_wf_max_safe(T_h);
    const double mwf      = std::clamp(m_dot_wf, p_.m_dot_wf_min, mwf_safe);
    const double mf       = std::clamp(m_dot_f, p_.m_dot_f_min, p_.m_dot_f_max);

    // Collector ODE:
    // dT_coll/dt = [eta_coll*A_coll*G - U_coll*A_coll*(T_coll-T_amb)
    //               - m_dot_f*cp_f*(T_coll - T_amb)] / (m_coll*cp_f)
    const double Q_solar  = p_.eta_coll * p_.A_coll * G;
    const double Q_loss_c = p_.U_coll * p_.A_coll * (T_coll - T_amb);
    const double Q_xfer_c = mf * p_.cp_f * (T_coll - T_h);  // heat to tank

    const double dTcoll = (Q_solar - Q_loss_c - Q_xfer_c) / (p_.m_coll * p_.cp_f);

    // Tank ODE:
    // dT_h/dt = [m_dot_f*cp_f*(T_coll - T_h) - Q_evap] / (m_tank*cp_f)
    const double Q_evap = Q_evap_from_state(T_h, mwf, T_c);
    const double dTh    = (Q_xfer_c - Q_evap) / (p_.m_tank * p_.cp_f);

    // Forward Euler
    x_(0) = T_h    + p_.Ts * dTh;
    x_(1) = T_coll + p_.Ts * dTcoll;

    // Clamp to physically reasonable range
    x_(0) = std::max(T_c + 1.0, x_(0));
    x_(1) = std::max(T_amb, x_(1));
}

} // namespace sotec
