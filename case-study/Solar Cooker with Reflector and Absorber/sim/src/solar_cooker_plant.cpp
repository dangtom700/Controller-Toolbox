#include "solar_cooker_plant.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>
#include <cmath>
#include <algorithm>

namespace cooker {

using json = nlohmann::json;

// ---------------------------------------------------------------------------
// PlantParams::fromJson
// ---------------------------------------------------------------------------
PlantParams PlantParams::fromJson(const std::string& path)
{
    std::ifstream f(path);
    if (!f.is_open())
        throw std::runtime_error("Cannot open plant JSON: " + path);
    json j; f >> j;

    PlantParams p;
    if (j.contains("A_box"))        p.A_box       = j["A_box"].get<double>();
    if (j.contains("A_abs"))        p.A_abs       = j["A_abs"].get<double>();
    if (j.contains("A_c"))          p.A_c         = j["A_c"].get<double>();
    if (j.contains("A_pot"))        p.A_pot       = j["A_pot"].get<double>();
    if (j.contains("eta_box"))      p.eta_box     = j["eta_box"].get<double>();
    if (j.contains("eta_TBPR"))     p.eta_TBPR    = j["eta_TBPR"].get<double>();
    if (j.contains("m_abs"))        p.m_abs       = j["m_abs"].get<double>();
    if (j.contains("cp_abs"))       p.cp_abs      = j["cp_abs"].get<double>();
    if (j.contains("m_pot"))        p.m_pot       = j["m_pot"].get<double>();
    if (j.contains("cp_pot"))       p.cp_pot      = j["cp_pot"].get<double>();
    if (j.contains("m_pcm"))        p.m_pcm       = j["m_pcm"].get<double>();
    if (j.contains("lambda_pcm"))   p.lambda_pcm  = j["lambda_pcm"].get<double>();
    if (j.contains("T_melt"))       p.T_melt      = j["T_melt"].get<double>();
    if (j.contains("delta_T_pcm"))  p.delta_T_pcm = j["delta_T_pcm"].get<double>();
    if (j.contains("h_abs_amb"))    p.h_abs_amb   = j["h_abs_amb"].get<double>();
    if (j.contains("h_abs_pot"))    p.h_abs_pot   = j["h_abs_pot"].get<double>();
    if (j.contains("U_loss"))       p.U_loss      = j["U_loss"].get<double>();
    if (j.contains("eps_abs"))      p.eps_abs     = j["eps_abs"].get<double>();
    if (j.contains("sigma"))        p.sigma       = j["sigma"].get<double>();
    if (j.contains("T_abs_max"))    p.T_abs_max   = j["T_abs_max"].get<double>();
    if (j.contains("Ts"))           p.Ts          = j["Ts"].get<double>();
    if (j.contains("duration"))     p.duration    = j["duration"].get<double>();
    return p;
}

// ---------------------------------------------------------------------------
// ScenarioConfig::fromJson
// ---------------------------------------------------------------------------
ScenarioConfig ScenarioConfig::fromJson(const std::string& path)
{
    std::ifstream f(path);
    if (!f.is_open())
        throw std::runtime_error("Cannot open scenario JSON: " + path);
    json j; f >> j;

    ScenarioConfig s;
    if (j.contains("id"))           s.id          = j["id"].get<std::string>();
    if (j.contains("description"))  s.description = j["description"].get<std::string>();
    if (j.contains("T_ref"))        s.T_ref       = j["T_ref"].get<double>();
    if (j.contains("T_abs_init"))   s.T_abs_init  = j["T_abs_init"].get<double>();
    if (j.contains("T_pot_init"))   s.T_pot_init  = j["T_pot_init"].get<double>();
    if (j.contains("duration_s"))   s.duration_s  = j["duration_s"].get<double>();
    if (j.contains("g_type"))       s.g_type      = j["g_type"].get<std::string>();
    if (j.contains("G_0"))          s.G_0         = j["G_0"].get<double>();
    if (j.contains("G_1"))          s.G_1         = j["G_1"].get<double>();
    if (j.contains("t_G_step"))     s.t_G_step    = j["t_G_step"].get<double>();
    if (j.contains("G_amp"))        s.G_amp       = j["G_amp"].get<double>();
    if (j.contains("G_freq"))       s.G_freq      = j["G_freq"].get<double>();
    if (j.contains("T_amb_0"))      s.T_amb_0     = j["T_amb_0"].get<double>();
    if (j.contains("v_type"))       s.v_type      = j["v_type"].get<std::string>();
    if (j.contains("V_wind_0"))     s.V_wind_0    = j["V_wind_0"].get<double>();
    if (j.contains("V_wind_1"))     s.V_wind_1    = j["V_wind_1"].get<double>();
    if (j.contains("t_wind_step"))  s.t_wind_step = j["t_wind_step"].get<double>();
    return s;
}

double ScenarioConfig::GAt(double t) const
{
    if (g_type == "step") {
        return (t_G_step >= 0.0 && t >= t_G_step) ? G_1 : G_0;
    } else if (g_type == "sinusoidal") {
        return G_0 + G_amp * std::sin(G_freq * t);
    }
    return G_0;  // constant
}

double ScenarioConfig::VWindAt(double t) const
{
    if (v_type == "step") {
        return (t_wind_step >= 0.0 && t >= t_wind_step) ? V_wind_1 : V_wind_0;
    }
    return V_wind_0;
}

Disturbance ScenarioConfig::distAt(double t) const
{
    return Disturbance{ GAt(t), T_amb_0, VWindAt(t) };
}

// ---------------------------------------------------------------------------
// SolarCookerPlant
// ---------------------------------------------------------------------------
SolarCookerPlant::SolarCookerPlant(const PlantParams& p)
    : p_(p), x_(Eigen::Vector2d(p.T_abs_max * 0.0, 30.0))  // dummy init; reset() below
{
    reset();
}

void SolarCookerPlant::reset(double T_abs_init, double T_pot_init)
{
    x_(0) = T_abs_init;
    x_(1) = T_pot_init;
}

double SolarCookerPlant::C_eff(double T_abs) const
{
    const double T_lo = p_.T_melt - 0.5 * p_.delta_T_pcm;
    const double T_hi = p_.T_melt + 0.5 * p_.delta_T_pcm;
    if (T_abs >= T_lo && T_abs <= T_hi)
        return p_.m_abs * p_.cp_abs + p_.m_pcm * p_.lambda_pcm / p_.delta_T_pcm;
    return p_.m_abs * p_.cp_abs;
}

double SolarCookerPlant::phiPCM() const
{
    const double T_lo = p_.T_melt - 0.5 * p_.delta_T_pcm;
    return std::clamp((x_(0) - T_lo) / p_.delta_T_pcm, 0.0, 1.0);
}

double SolarCookerPlant::Q_absorbed(double f_shade, double G) const
{
    return (1.0 - f_shade) * (p_.eta_box + p_.eta_TBPR) * G * p_.A_box;
}

Eigen::Vector2d SolarCookerPlant::ode(const Eigen::Vector2d& x,
                                       double f_shade,
                                       const Disturbance& d) const
{
    const double T_abs = x(0);
    const double T_pot = x(1);
    const double G     = std::max(d.G, 0.0);
    const double T_amb = d.T_amb;
    const double V     = d.V_wind;

    // Solar gain
    const double Q_solar = (1.0 - f_shade) * (p_.eta_box + p_.eta_TBPR) * G * p_.A_box;

    // Convective losses: still-air + wind
    const double h_wind  = 5.0 + 4.0 * V;
    const double Q_conv  = (p_.h_abs_amb + h_wind) * p_.A_abs * (T_abs - T_amb);

    // Absorber <-> pot coupling
    const double Q_pot_coupling = p_.h_abs_pot * p_.A_c * (T_abs - T_pot);

    // Radiation loss (sky temperature approximation)
    const double T_sky_K = (T_amb - 10.0) + 273.15;
    const double T_abs_K = T_abs + 273.15;
    const double Q_rad   = p_.eps_abs * p_.sigma * p_.A_abs
                           * (T_abs_K * T_abs_K * T_abs_K * T_abs_K
                              - T_sky_K * T_sky_K * T_sky_K * T_sky_K);

    // Pot energy balance
    const double Q_pot_loss = p_.U_loss * p_.A_pot * (T_pot - T_amb);

    const double C_abs = C_eff(T_abs);
    const double C_pot = p_.m_pot * p_.cp_pot;

    Eigen::Vector2d dx;
    dx(0) = (Q_solar - Q_conv - Q_pot_coupling - Q_rad) / C_abs;
    dx(1) = (Q_pot_coupling - Q_pot_loss) / C_pot;
    return dx;
}

void SolarCookerPlant::step(double f_shade, const Disturbance& d)
{
    const double h = p_.Ts / static_cast<double>(N_SUBSTEPS);

    for (int i = 0; i < N_SUBSTEPS; ++i) {
        const Eigen::Vector2d k1 = ode(x_,               f_shade, d);
        const Eigen::Vector2d k2 = ode(x_ + 0.5*h*k1,  f_shade, d);
        const Eigen::Vector2d k3 = ode(x_ + 0.5*h*k2,  f_shade, d);
        const Eigen::Vector2d k4 = ode(x_ + h*k3,       f_shade, d);
        x_ += (h / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);
    }

    // Hard clamp on absorber temperature
    x_(0) = std::min(x_(0), p_.T_abs_max);
    // Prevent pot from going below ambient (numerical sanity)
    // T_pot physically bounded; allow slight undershoot without hard clamp
}

} // namespace cooker
