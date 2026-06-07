#include "susp_plant.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>
#include <cmath>
#include <algorithm>

using json = nlohmann::json;

namespace susp {

// ---------------------------------------------------------------------------
// PlantParams
// ---------------------------------------------------------------------------

void PlantParams::init()
{
    omega_body  = std::sqrt(k_s / m_s);
    omega_wheel = std::sqrt((k_s + k_t) / m_u);
    zeta_body   = c_s / (2.0 * std::sqrt(k_s * m_s));
}

PlantParams PlantParams::fromJson(const std::string& path)
{
    std::ifstream f(path);
    if (!f.is_open())
        throw std::runtime_error("Cannot open plant params: " + path);
    json cfg = json::parse(f);

    PlantParams p;
    p.m_s      = cfg.value("m_s",      240.0);
    p.m_u      = cfg.value("m_u",       36.0);
    p.k_s      = cfg.value("k_s",    16000.0);
    p.c_s      = cfg.value("c_s",      980.0);
    p.k_t      = cfg.value("k_t",   160000.0);
    p.F_max    = cfg.value("F_max",   2000.0);
    p.Ts       = cfg.value("Ts",       0.005);
    p.duration = cfg.value("duration",   5.0);
    p.init();
    return p;
}

// ---------------------------------------------------------------------------
// QuarterCarPlant
// ---------------------------------------------------------------------------

QuarterCarPlant::QuarterCarPlant(const PlantParams& p)
    : p_(p)
    , x_(Eigen::Vector4d::Zero())
{}

void QuarterCarPlant::reset()
{
    x_.setZero();
}

// RK4 derivative: xdot = f(x, F_act, z_r)
Eigen::Vector4d QuarterCarPlant::f(const Eigen::Vector4d& x,
                                    double F_act,
                                    double z_r) const
{
    const double z_s  = x(0), dz_s = x(1);
    const double z_u  = x(2), dz_u = x(3);

    const double defl    = z_s - z_u;
    const double d_defl  = dz_s - dz_u;
    const double tyre    = z_u - z_r;

    const double ddz_s = (-p_.k_s * defl - p_.c_s * d_defl + F_act) / p_.m_s;
    const double ddz_u = ( p_.k_s * defl + p_.c_s * d_defl
                           - p_.k_t * tyre - F_act) / p_.m_u;

    return Eigen::Vector4d(dz_s, ddz_s, dz_u, ddz_u);
}

void QuarterCarPlant::step(double F_act, double z_r)
{
    const double h = p_.Ts;
    const Eigen::Vector4d k1 = f(x_,                 F_act, z_r);
    const Eigen::Vector4d k2 = f(x_ + 0.5*h*k1,     F_act, z_r);
    const Eigen::Vector4d k3 = f(x_ + 0.5*h*k2,     F_act, z_r);
    const Eigen::Vector4d k4 = f(x_ + h*k3,          F_act, z_r);
    x_ += (h / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);
}

double QuarterCarPlant::bodyAccel(double F_act, double z_r) const
{
    const double defl   = x_(0) - x_(2);
    const double d_defl = x_(1) - x_(3);
    return (-p_.k_s * defl - p_.c_s * d_defl + F_act) / p_.m_s;
}

} // namespace susp
