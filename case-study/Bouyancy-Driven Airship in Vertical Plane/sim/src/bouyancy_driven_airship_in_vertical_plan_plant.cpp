#include "bouyancy_driven_airship_in_vertical_plan_plant.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>
#include <cmath>
#include <algorithm>

namespace bouyancydrivenairshipinverticalplan {

PlantParams PlantParams::fromJson(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("cannot open " + path);
    nlohmann::json j; f >> j;
    PlantParams p;
    p.m_bar    = j.value("m_bar", p.m_bar);
    p.ms       = j.value("ms", p.ms);
    p.J        = j.value("J", p.J);
    p.rp3      = j.value("rp3", p.rp3);
    p.g        = j.value("g", p.g);
    p.u_max    = j.value("u_max", p.u_max);
    p.u_min    = j.value("u_min", p.u_min);
    p.rp1_max  = j.value("rp1_max", p.rp1_max);
    p.rp1_min  = j.value("rp1_min", p.rp1_min);
    p.Ts       = j.value("Ts", p.Ts);
    p.duration = j.value("duration", p.duration);
    return p;
}

RhoSigma rhoSigma(const PlantParams& p, double theta, double q, double rp1, double w, double u) {
    const double denom = p.J + p.m_bar * rp1 * rp1;
    const double rho1 = -(1.0 / denom) * (p.m_bar * p.rp3 * rp1 * q * q
                                           + 2.0 * p.m_bar * rp1 * w * q
                                           + p.m_bar * p.g * rp1 * std::cos(theta)
                                           + p.rp3 * u);
    const double sigma1 = (1.0 / denom) * ((p.J * rp1 + p.m_bar * rp1 * rp1 * rp1
                                             + p.m_bar * p.rp3 * p.rp3 * rp1) * q * q
                                            + 2.0 * p.m_bar * p.rp3 * rp1 * w * q
                                            - denom * p.g * std::sin(theta)
                                            + p.m_bar * p.g * p.rp3 * rp1 * std::cos(theta)
                                            + (p.J / p.m_bar + rp1 * rp1 + p.rp3 * p.rp3) * u);
    return RhoSigma{rho1, sigma1};
}

State ode(const PlantParams& p, const State& x, double u, double m0) {
    const double theta = x(THETA), q = x(Q), rp1 = x(RP1), w = x(W), v1 = x(V1), v3 = x(V3);

    const RhoSigma rs = rhoSigma(p, theta, q, rp1, w, u);
    const double denom = p.J + p.m_bar * rp1 * rp1;

    // theta_ddot / v3_dot are mutually dependent (m_bar*rp1*theta_ddot term inside v3_dot) -
    // solved here as the closed-form 2x2 linear system from README "Implicit coupling".
    const double A = p.m_bar * rp1 / denom;
    const double B = p.m_bar * rp1 / (p.ms + p.m_bar);
    const double C = (1.0 / (p.ms + p.m_bar)) * ((p.ms + p.m_bar) * q * v1
                                                   + m0 * p.g * std::cos(theta)
                                                   + p.m_bar * p.rp3 * q * q
                                                   + 2.0 * p.m_bar * q * w);

    const double v3_dot     = (C + B * rs.rho1 - A * B * q * v1) / (1.0 - A * B);
    const double theta_ddot = rs.rho1 + A * v3_dot - A * q * v1;

    const double v1_dot = (1.0 / p.ms) * (-p.ms * q * v3 + (p.m_bar - m0) * p.g * std::sin(theta) - u);

    const double rp1_ddot = rs.sigma1 + (1.0 / denom) * (p.m_bar * rp1 * p.rp3 * (q * v1 - v3_dot)
                                                           - denom * (q * v3 + v1_dot));

    State xdot;
    xdot(THETA) = q;
    xdot(Q)     = theta_ddot;
    xdot(RP1)   = w;
    xdot(W)     = rp1_ddot;
    xdot(V1)    = v1_dot;
    xdot(V3)    = v3_dot;
    return xdot;
}

Eigen::Vector4d fixedCenterOde(const PlantParams& p, const Eigen::Vector4d& z, double u) {
    const RhoSigma rs = rhoSigma(p, z(0), z(1), z(2), z(3), u);
    Eigen::Vector4d zdot;
    zdot << z(1), rs.rho1, z(3), rs.sigma1;
    return zdot;
}

double trimInput(const PlantParams& p, double theta_ref, double rp1_ref) {
    return -p.m_bar * p.g * rp1_ref * std::cos(theta_ref) / p.rp3;
}

Plant::Plant(const PlantParams& p) : p_(p) {
    x_.setZero();
}

void Plant::reset(double theta0, double rp1_0, double v1_0, double v3_0) {
    x_(THETA) = theta0;
    x_(Q)     = 0.0;
    x_(RP1)   = rp1_0;
    x_(W)     = 0.0;
    x_(V1)    = v1_0;
    x_(V3)    = v3_0;
}

void Plant::step(double u, double m0) {
    const double u_c = std::clamp(u, p_.u_min, p_.u_max);
    const double h = p_.Ts;

    const State k1 = ode(p_, x_, u_c, m0);
    const State k2 = ode(p_, x_ + 0.5 * h * k1, u_c, m0);
    const State k3 = ode(p_, x_ + 0.5 * h * k2, u_c, m0);
    const State k4 = ode(p_, x_ + h * k3, u_c, m0);
    x_ += (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

    // Defensive mechanical-stop guard - see README "Model simplifications".
    x_(RP1) = std::clamp(x_(RP1), p_.rp1_min, p_.rp1_max);
}

}  // namespace bouyancydrivenairshipinverticalplan
