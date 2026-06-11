#include "smismo_plant.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <cmath>
#include <stdexcept>
#include <algorithm>

namespace smismo {

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
    p.A1      = get("A1", p.A1);
    p.A2      = get("A2", p.A2);
    p.V10     = get("V10", p.V10);
    p.V20     = get("V20", p.V20);
    p.stroke  = get("stroke", p.stroke);
    p.m       = get("m", p.m);
    p.g       = get("g", p.g);
    p.beta_e  = get("beta_e", p.beta_e);
    p.rho     = get("rho", p.rho);
    p.C_le    = get("C_le", p.C_le);
    p.P_s     = get("P_s", p.P_s);
    p.P_r     = get("P_r", p.P_r);
    p.Q_nom1  = get("Q_nom1", p.Q_nom1);
    p.Q_nom2  = get("Q_nom2", p.Q_nom2);
    p.DP_nom  = get("DP_nom", p.DP_nom);
    p.w_v1    = get("w_v1", p.w_v1);
    p.xi_v1   = get("xi_v1", p.xi_v1);
    p.w_v2    = get("w_v2", p.w_v2);
    p.xi_v2   = get("xi_v2", p.xi_v2);
    p.u_max   = get("u_max", p.u_max);
    p.v_eps   = get("v_eps", p.v_eps);
    p.P_bd    = get("P_bd", p.P_bd);
    p.P_min   = get("P_min", p.P_min);
    p.P_max   = get("P_max", p.P_max);
    p.Ts      = get("Ts", p.Ts);
    p.n_sub   = static_cast<int>(get("n_sub", p.n_sub));
    p.duration = get("duration", p.duration);
    return p;
}

// ---------------------------------------------------------------------------
// ScenarioConfig
// ---------------------------------------------------------------------------
double ScenarioConfig::xRefAt(double t) const {
    if (ref_type == "paper_sine") {
        // Chen et al. (2018) Sec. 5: x_Ld = 0.25 + 0.25*sin(pi*t/2 - pi/2)
        return 0.25 + 0.25 * std::sin(M_PI * t / 2.0 - M_PI / 2.0);
    }
    if (ref_type == "step" && t_step >= 0.0 && t >= t_step)
        return x_ref_1;
    return x_ref_0;  // "hold" and pre-step
}

double ScenarioConfig::fExtAt(double t) const {
    if (t_F_step >= 0.0 && t >= t_F_step) return F_1;
    return F_0;
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
    s.duration_s  = getd("duration_s", s.duration_s);
    s.x_init      = getd("x_init", s.x_init);
    s.P1_init     = getd("P1_init", s.P1_init);
    s.P2_init     = getd("P2_init", s.P2_init);
    s.ref_type    = gets("ref_type", s.ref_type);
    s.x_ref_0     = getd("x_ref_0", s.x_ref_0);
    s.x_ref_1     = getd("x_ref_1", s.x_ref_1);
    s.t_step      = getd("t_step", s.t_step);
    s.F_0         = getd("F_0", s.F_0);
    s.F_1         = getd("F_1", s.F_1);
    s.t_F_step    = getd("t_F_step", s.t_F_step);
    return s;
}

// ---------------------------------------------------------------------------
// SmismoPlant
// ---------------------------------------------------------------------------
SmismoPlant::SmismoPlant(const PlantParams& p) : p_(p) {
    reset(0.05, 2.5e6, 2.0e6);
}

void SmismoPlant::reset(double x_init, double P1_init, double P2_init) {
    x_.setZero();
    x_(0) = std::clamp(x_init, 0.0, p_.stroke);
    x_(2) = P1_init;
    x_(3) = P2_init;
    Q_s_last_ = 0.0;
    energy_J_ = 0.0;
}

double SmismoPlant::flowSqrt(double dp) const {
    // Signed, smooth at dp=0: dp/sqrt(|dp|+eps) -> sgn(dp)*sqrt(|dp|) for |dp|>>eps.
    // Negative dp gives reverse flow (anti-cavitation backfill from tank/supply).
    constexpr double dp_eps = 1.0e3;  // [Pa]
    return dp / std::sqrt(std::abs(dp) + dp_eps);
}

double SmismoPlant::orificeFlow(double xv, double P_chamber, double Q_nom) const {
    // Chen Eq. 4-5 in Liu Eq. 14 rated-flow form (signed ingress flow):
    //   xv >= 0: chamber fed from supply, DP = P_s - P_chamber
    //   xv <  0: chamber discharges to tank, DP = P_chamber - P_r
    const double K_q = Q_nom / std::sqrt(p_.DP_nom);
    if (xv >= 0.0)
        return K_q * xv * flowSqrt(p_.P_s - P_chamber);
    return K_q * xv * flowSqrt(P_chamber - p_.P_r);
}

double SmismoPlant::Q1() const { return orificeFlow(x_(4), x_(2), p_.Q_nom1); }
double SmismoPlant::Q2() const { return orificeFlow(x_(6), x_(3), p_.Q_nom2); }

double SmismoPlant::frictionForce(double v) const {
    // Chen Sec. 5.1 identified Stribeck friction [N]:
    //   v > 0:  68 + 13 v + 11 exp(-|3v/0.5|)
    //   v < 0: -79 + 24 v - 16 exp(-|3v/0.6|)
    // Regularised: linear through the static band |v| < v_eps.
    auto f_pos = [](double v_) { return  68.0 + 13.0 * v_ + 11.0 * std::exp(-std::abs(3.0 * v_ / 0.5)); };
    auto f_neg = [](double v_) { return -79.0 + 24.0 * v_ - 16.0 * std::exp(-std::abs(3.0 * v_ / 0.6)); };
    const double ve = p_.v_eps;
    if (v >  ve) return f_pos(v);
    if (v < -ve) return f_neg(v);
    const double fp = f_pos(ve);
    const double fn = f_neg(-ve);
    return fn + (v + ve) / (2.0 * ve) * (fp - fn);
}

SmismoPlant::State SmismoPlant::deriv(const State& x, double u1, double u2,
                                      double F_ext) const {
    const double x_L  = x(0);
    const double v_L  = x(1);
    const double P1   = x(2);
    const double P2   = x(3);
    const double xv1  = x(4);
    const double dxv1 = x(5);
    const double xv2  = x(6);
    const double dxv2 = x(7);

    // Orifice flows (signed ingress, Chen Eq. 4-5)
    const double Q1 = orificeFlow(xv1, P1, p_.Q_nom1);
    const double Q2 = orificeFlow(xv2, P2, p_.Q_nom2);

    // Cross-port leakage (DQ residual, Chen Eq. 3)
    const double Q_le = p_.C_le * (P1 - P2);

    // Chamber volumes (position-dependent)
    const double V1 = p_.V10 + p_.A1 * x_L;
    const double V2 = p_.V20 - p_.A2 * x_L;

    // Load dynamics (Chen Eq. 2): hanging load, extension positive
    const double F_f = frictionForce(v_L);
    const double dv  = (P1 * p_.A1 - P2 * p_.A2 - p_.m * p_.g - F_f - F_ext) / p_.m;

    // Continuity (Chen Eq. 3)
    const double dP1 = p_.beta_e / V1 * (Q1 - p_.A1 * v_L - Q_le);
    const double dP2 = p_.beta_e / V2 * (Q2 + p_.A2 * v_L + Q_le);

    // Valve spool 2nd-order dynamics (Chen Eq. 7), normalised DC gain u/u_max
    const double ddxv1 = p_.w_v1 * p_.w_v1 * (u1 / p_.u_max - xv1)
                       - 2.0 * p_.xi_v1 * p_.w_v1 * dxv1;
    const double ddxv2 = p_.w_v2 * p_.w_v2 * (u2 / p_.u_max - xv2)
                       - 2.0 * p_.xi_v2 * p_.w_v2 * dxv2;

    State dx;
    dx << v_L, dv, dP1, dP2, dxv1, ddxv1, dxv2, ddxv2;
    return dx;
}

void SmismoPlant::step(double u1, double u2, double F_ext) {
    u1 = std::clamp(u1, -p_.u_max, p_.u_max);
    u2 = std::clamp(u2, -p_.u_max, p_.u_max);

    const double dt = p_.Ts / std::max(1, p_.n_sub);

    for (int s = 0; s < std::max(1, p_.n_sub); ++s) {
        const State k1 = deriv(x_, u1, u2, F_ext);
        const State k2 = deriv(x_ + 0.5 * dt * k1, u1, u2, F_ext);
        const State k3 = deriv(x_ + 0.5 * dt * k2, u1, u2, F_ext);
        const State k4 = deriv(x_ + dt * k3, u1, u2, F_ext);
        x_ += dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

        // Pressure clamps (vapor / burst)
        x_(2) = std::clamp(x_(2), p_.P_min, p_.P_max);
        x_(3) = std::clamp(x_(3), p_.P_min, p_.P_max);

        // Spool saturation
        x_(4) = std::clamp(x_(4), -1.0, 1.0);
        x_(6) = std::clamp(x_(6), -1.0, 1.0);

        // End stops: clamp position and zero velocity on contact
        if (x_(0) <= 0.0)        { x_(0) = 0.0;        if (x_(1) < 0.0) x_(1) = 0.0; }
        if (x_(0) >= p_.stroke)  { x_(0) = p_.stroke;  if (x_(1) > 0.0) x_(1) = 0.0; }

        // Supply energy accounting: flow drawn from supply by either valve
        const double q1 = Q1();
        const double q2 = Q2();
        double Q_s = 0.0;
        if (x_(4) > 0.0) Q_s += std::max(0.0, q1);
        if (x_(6) > 0.0) Q_s += std::max(0.0, q2);
        Q_s_last_  = Q_s;
        energy_J_ += p_.P_s * Q_s * dt;
    }
}

} // namespace smismo
