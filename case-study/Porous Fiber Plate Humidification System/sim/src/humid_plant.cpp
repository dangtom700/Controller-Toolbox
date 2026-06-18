#include "humid_plant.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>
#include <algorithm>
#include <cmath>

using json = nlohmann::json;
using namespace humid::psychro;

namespace humid {

// ---------------------------------------------------------------------------
// PlantParams JSON loader
// ---------------------------------------------------------------------------

PlantParams PlantParams::fromJson(const std::string& path)
{
    std::ifstream f(path);
    if (!f.is_open())
        throw std::runtime_error("HumidPlant: cannot open config: " + path);
    json cfg = json::parse(f);

    PlantParams p{};
    auto& pl = cfg["plates"];
    p.l_plate   = pl["l_plate_m"];
    p.A_plates  = pl["A_plates_m2"];
    p.C_gap     = pl["C_gap"];

    auto& rm = cfg["room"];
    p.V_room    = rm["V_room_m3"];
    p.rho_a     = rm["rho_air_kgm3"];
    p.T_room_K  = rm["T_room_C"].get<double>() + 273.15;
    p.ACH       = rm["ACH"];
    p.G_occ_per = rm["G_occ_per_person_gh"];

    auto& sim = cfg["simulation"];
    p.Ts       = sim["dt_s"];
    p.duration = sim["duration_s"];

    p.init();
    return p;
}

// ---------------------------------------------------------------------------
// HumidifierPhysics
// ---------------------------------------------------------------------------

double HumidifierPhysics::compute(double u_fan,
                                   double Ta_K,
                                   double phi_in,
                                   PlantOutput& out) const
{
    // Clamp fan speed to valid range
    u_fan = std::clamp(u_fan, 1.0, 3.5);

    // Wet-bulb temperature of inlet air = plate surface temperature
    double Tp_K = wetBulb(Ta_K, phi_in);
    double Tm_K = 0.5 * (Ta_K + Tp_K);

    // Diffusivity and transport properties at mean boundary-layer temperature
    double D    = D_wv(Tm_K);
    double nu   = nu_air(Tm_K);
    double Sc   = nu / D;

    // Gap velocity (CFD correction factor, paper Table 2)
    double u_gap = p_.C_gap * u_fan;
    double Re    = u_gap * p_.l_plate / nu;

    // Sherwood number (laminar flat-plate, Re < 1e5 for all conditions)
    double Sh = 0.664 * std::sqrt(Re) * std::cbrt(Sc);
    double hm = Sh * D / p_.l_plate;

    // Vapour pressure at plate surface and in bulk air
    double Pps = Psat(Tp_K);
    double Pa  = phi_in * Psat(Ta_K);

    // Mass flux: mw = hm * A * (rho_surface - rho_air)
    //   rho = P / (Rw * T)  ->  delta_rho = Pps/(Rw*Tp) - Pa/(Rw*Ta)
    double delta_rho = Pps / (kRw * Tp_K) - Pa / (kRw * Ta_K);
    double mw_kgs    = hm * p_.A_plates * delta_rho;  // [kg/s]
    double H_gh      = std::max(0.0, mw_kgs * 3.6e6); // [g/h]

    out.Tp_K = Tp_K;
    out.Re   = Re;
    out.Sh   = Sh;
    out.hm   = hm;
    out.H_gh = H_gh;
    return H_gh;
}

// ---------------------------------------------------------------------------
// HumidificationPlant
// ---------------------------------------------------------------------------

HumidificationPlant::HumidificationPlant(const PlantParams& p)
    : p_(p), hum_(p), omega_room_(0.0), delay_idx_(0)
{
    delay_buf_.fill(0.0);
    p_.init();
    reset();
}

HumidificationPlant HumidificationPlant::fromJson(const std::string& path)
{
    return HumidificationPlant(PlantParams::fromJson(path));
}

void HumidificationPlant::reset(double phi_room_init)
{
    // Initialise omega from phi_room at T_room
    omega_room_ = omega_gkg_from_phi(phi_room_init, p_.T_room_K);

    double phi_sens = phi_from_omega(omega_room_, p_.T_room_K);
    delay_buf_.fill(phi_sens);
    delay_idx_ = 0;
    last_ = {};
    last_.phi_room    = phi_room_init;
    last_.phi_measured = phi_room_init;
    last_.omega_room  = omega_room_;
}

PlantOutput HumidificationPlant::step(const ControlInput& u, const Disturbance& d)
{
    PlantOutput out{};

    // Clamp inputs
    double u_fan  = std::clamp(u.u_fan,  1.0, 3.5);
    double Ta_K   = std::clamp(u.Ta_sp, 303.15, 323.15);  // [30, 50]^\circC

    // After the heater raises room air from T_room to Ta, relative humidity drops:
    //   phi_in = phi_room * Psat(T_room) / Psat(Ta)
    double phi_room  = phi_from_omega(omega_room_, p_.T_room_K);
    double phi_in    = phi_room * Psat(p_.T_room_K) / Psat(Ta_K);
    phi_in = std::clamp(phi_in, 0.0, 1.0);

    out.phi_in = phi_in;

    // Humidifier produces H [g/h]
    double H_gh = hum_.compute(u_fan, Ta_K, phi_in, out);

    // Outdoor specific humidity [g/kg]
    double Pa_out  = d.phi_out * Psat(d.T_out_K);
    double omega_out = 1000.0 * 0.622 * Pa_out / std::max(kPatm - Pa_out, 1.0);

    // Room moisture balance (forward Euler):
    //   d(omega)/dt = [H_gs + G_occ_gs + m_dot_inf*(omega_out - omega)] / (rho_a*V_room)
    double H_gs      = H_gh / 3600.0;                               // [g/s]
    double G_occ_gs  = d.n_occ * p_.G_occ_per / 3600.0;            // [g/s]
    double rhoV      = p_.rho_a * p_.V_room;                         // [kg]

    double domega_dt = (H_gs + G_occ_gs
                        + p_.m_dot_inf * (omega_out - omega_room_)) / rhoV;

    omega_room_ += p_.Ts * domega_dt;
    omega_room_  = std::max(omega_room_, 0.0);

    // Convert to relative humidity
    phi_room = phi_from_omega(omega_room_, p_.T_room_K);
    phi_room = std::clamp(phi_room, 0.0, 0.99);

    // Sensor delay (2-step FIFO)
    double phi_true = phi_room;
    double phi_del  = delay_buf_[delay_idx_];
    delay_buf_[delay_idx_] = phi_true;
    delay_idx_ = (delay_idx_ + 1) % kDelay;

    out.phi_room     = phi_true;
    out.phi_measured = phi_del;
    out.omega_room   = omega_room_;

    last_ = out;
    return out;
}

} // namespace humid
