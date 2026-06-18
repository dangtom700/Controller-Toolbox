#pragma once
#include "psychrometrics.h"
#include <string>
#include <array>
#include <cmath>

// Porous-fiber-plate humidification plant model.
//
// Source: Ye, Yan, Ni - Applied Thermal Engineering 245 (2024) 122877
//         "Theoretical calculation and experimental analysis on
//          humidification performance of high-performance water-absorbing
//          porous fiber plates"
//
// Architecture:
//   Subsystem 1 (algebraic) - humidifier physics
//     Maps (u_fan, Ta_K, phi_in) -> H [g/h] using the laminar flat-plate
//     criterion correlation (Sherwood, Schmidt, Reynolds, Eq. 4-8).
//     phi_in is derived from phi_room by accounting for the heater raising
//     room air from T_room to Ta_setpoint.
//
//   Subsystem 2 (ODE, Euler) - room humidity dynamics
//     Integrates specific humidity omega_room [g/kg] via a moisture balance
//     including the humidifier output, infiltration, and occupant generation.
//
//   Sensor model - 2-step FIFO delay on the measured phi_room output to
//     model the 60-s transport lag of a typical duct RH sensor.

namespace humid {

// ---------------------------------------------------------------------------
// Parameter group
// ---------------------------------------------------------------------------

struct PlantParams {
    // Plate geometry (paper Sec. 2.2 / Table 3)
    double l_plate   = 0.100;    // flow-direction depth of each plate [m]
    double A_plates  = 0.400;    // total plate area (20 * 2 sides * 0.1 * 0.1) [m^2]
    double C_gap     = 1.25;     // gap-to-duct velocity ratio (CFD, Table 2)

    // Room
    double V_room    = 50.0;     // room volume [m^3]
    double rho_a     = 1.2;      // dry air density [kg/m^3]
    double T_room_K  = 295.15;   // room dry-bulb temperature (fixed, 22^\circC) [K]
    double ACH       = 0.5;      // infiltration air changes per hour [h^-1]
    double G_occ_per = 50.0;     // moisture generation per occupant [g/h per person]

    // Simulation
    double Ts        = 30.0;     // controller sample time [s]
    double duration  = 10800.0;  // scenario duration [s] (3 hours)

    // Derived (computed once after load)
    double m_dot_inf = 0.0;      // infiltration mass flow [kg/s] - filled by init()
    double tau_room  = 0.0;      // room moisture time constant [s] - filled by init()

    void init() {
        m_dot_inf = rho_a * V_room * ACH / 3600.0;
        tau_room  = rho_a * V_room / m_dot_inf;
    }

    static PlantParams fromJson(const std::string& path);
};

// ---------------------------------------------------------------------------
// Disturbance / scenario inputs
// ---------------------------------------------------------------------------

struct Disturbance {
    double T_out_K  = 263.15;  // outdoor dry-bulb temperature [K] (-10^\circC)
    double phi_out  = 0.35;    // outdoor relative humidity [-]
    double n_occ    = 0.0;     // number of occupants [-]
};

// ---------------------------------------------------------------------------
// Control input
// ---------------------------------------------------------------------------

struct ControlInput {
    double u_fan  = 2.25;   // fan speed [m/s], clamped to [1.0, 3.5]
    double Ta_sp  = 313.15; // heater setpoint [K] (40^\circC default)
};

// ---------------------------------------------------------------------------
// Plant output
// ---------------------------------------------------------------------------

struct PlantOutput {
    double phi_room;      // actual indoor relative humidity [-]
    double phi_measured;  // delayed sensor reading [-]
    double omega_room;    // specific humidity [g/kg]
    double phi_in;        // inlet RH to humidifier (after heating) [-]
    double H_gh;          // humidification capacity [g/h]
    double Tp_K;          // plate surface (wet-bulb) temperature [K]
    double Re;            // Reynolds number [-]
    double Sh;            // Sherwood number [-]
    double hm;            // convective mass-transfer coefficient [m/s]
};

// ---------------------------------------------------------------------------
// Humidifier physics (static, from paper Eqs. 2-8)
// ---------------------------------------------------------------------------

class HumidifierPhysics {
public:
    explicit HumidifierPhysics(const PlantParams& p) : p_(p) {}

    // Returns H [g/h] and fills diagnostics into out.
    // phi_in = relative humidity of air entering the humidifier element.
    // Returns 0 if condensation conditions (phi_in > phi at wet-bulb).
    double compute(double u_fan, double Ta_K, double phi_in, PlantOutput& out) const;

private:
    PlantParams p_;
};

// ---------------------------------------------------------------------------
// Full plant (humidifier + room + sensor delay)
// ---------------------------------------------------------------------------

class HumidificationPlant {
public:
    explicit HumidificationPlant(const PlantParams& p);

    static HumidificationPlant fromJson(const std::string& path);

    // Advance one Ts step. Returns plant output including delayed sensor reading.
    PlantOutput step(const ControlInput& u, const Disturbance& d);

    void reset(double phi_room_init = 0.20);

    const PlantParams& params()     const { return p_; }
    const PlantOutput& lastOutput() const { return last_; }

private:
    PlantParams      p_;
    HumidifierPhysics hum_;
    double           omega_room_;   // specific humidity state [g/kg]

    // 2-step sensor delay buffer (FIFO)
    static constexpr int kDelay = 2;
    std::array<double, kDelay> delay_buf_;
    int delay_idx_;

    PlantOutput last_{};
};

} // namespace humid
