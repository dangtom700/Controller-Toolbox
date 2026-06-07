#include "buck_boost_plant.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>
#include <cmath>

namespace conv {

using json = nlohmann::json;

PlantParams PlantParams::fromJson(const std::string& path)
{
    std::ifstream f(path);
    if (!f.is_open())
        throw std::runtime_error("Cannot open plant JSON: " + path);
    json j; f >> j;

    PlantParams p;
    if (j.contains("L"))          p.L          = j["L"].get<double>();
    if (j.contains("C"))          p.C          = j["C"].get<double>();
    if (j.contains("R"))          p.R          = j["R"].get<double>();
    if (j.contains("V_in_nom"))   p.V_in_nom   = j["V_in_nom"].get<double>();
    if (j.contains("f_s"))        p.f_s        = j["f_s"].get<double>();
    if (j.contains("Ts"))         p.Ts         = j["Ts"].get<double>();
    if (j.contains("d_min"))      p.d_min      = j["d_min"].get<double>();
    if (j.contains("d_max"))      p.d_max      = j["d_max"].get<double>();
    if (j.contains("hysteresis")) p.hysteresis = j["hysteresis"].get<double>();
    if (j.contains("duration"))   p.duration   = j["duration"].get<double>();
    return p;
}

BuckBoostPlant::BuckBoostPlant(const PlantParams& p)
    : p_(p), x_(Eigen::Vector2d::Zero())
{}

void BuckBoostPlant::reset()
{
    x_.setZero();
}

Eigen::Vector2d BuckBoostPlant::f(const Eigen::Vector2d& x, double d,
                                   ConvMode mode, double V_in) const
{
    const double i_L = x(0);
    const double v_C = std::max(x(1), 0.0);  // prevent negative voltage in integrator

    Eigen::Vector2d dx;
    if (mode == ConvMode::BUCK) {
        // L * di_L/dt = d*V_in - v_C
        // C * dv_C/dt = i_L - v_C/R
        dx(0) = (d * V_in - v_C) / p_.L;
        dx(1) = (i_L - v_C / p_.R) / p_.C;
    } else {
        // Boost:
        // L * di_L/dt = V_in - (1-d)*v_C
        // C * dv_C/dt = (1-d)*i_L - v_C/R
        const double one_minus_d = 1.0 - d;
        dx(0) = (V_in - one_minus_d * v_C) / p_.L;
        dx(1) = (one_minus_d * i_L - v_C / p_.R) / p_.C;
    }
    return dx;
}

void BuckBoostPlant::step(double d, ConvMode mode, double V_in)
{
    const double h = p_.Ts;
    const Eigen::Vector2d k1 = f(x_, d, mode, V_in);
    const Eigen::Vector2d k2 = f(x_ + 0.5*h*k1, d, mode, V_in);
    const Eigen::Vector2d k3 = f(x_ + 0.5*h*k2, d, mode, V_in);
    const Eigen::Vector2d k4 = f(x_ + h*k3,      d, mode, V_in);
    x_ += (h / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);

    // Clamp i_L >= 0 (discontinuous conduction mode boundary; averaged model holds in CCM)
    if (x_(0) < 0.0) x_(0) = 0.0;
    // Clamp v_C >= 0
    if (x_(1) < 0.0) x_(1) = 0.0;
}

} // namespace conv
