#pragma once
#include <Eigen/Dense>
#include <string>
#include <vector>

// Solar Ocean Thermal Energy Conversion (S-OTEC) plant model.
//
// Reference: Gao et al. (2024) Applied Thermal Engineering 245, 122776.
//
// 2 states: x = [T_h, T_coll]   [^\circC]
//   T_h    = hot reservoir (buffer tank) temperature
//   T_coll = solar collector outlet temperature
//
// Inputs: u = [m_dot_f, m_dot_wf]   [kg/s]
//   m_dot_f   = solar collector loop pump flow  [0.1, 0.5 kg/s]
//   m_dot_wf  = R134a working fluid pump flow   [0.01, 0.08 kg/s]
//
// Disturbances: G [W/m^2], T_amb [^\circC], T_c [^\circC] (cold ocean)
//
// HARD CONSTRAINT: P_inlet(T_h, m_dot_wf) <= 1.38 MPa.
//   m_dot_wf is automatically clamped before use.
//
// Integration: forward Euler at Ts = 30 s.

namespace sotec {

struct PlantParams {
    // Solar collector
    double A_coll     = 20.0;     // collector area [m^2]
    double eta_coll   = 0.65;     // collector efficiency
    double U_coll     = 3.0;      // collector overall loss [W/(m^2 K)]
    double m_coll     = 40.0;     // collector fluid mass [kg]
    double cp_f       = 4186.0;   // specific heat of water [J/(kg K)]

    // Hot reservoir (buffer tank)
    double m_tank     = 200.0;    // tank fluid mass [kg]

    // ORC working fluid (R134a)
    double cp_wf      = 1460.0;   // R134a vapor specific heat [J/(kg K)]
    double eta_exp    = 0.65;     // expander isentropic efficiency
    double eta_pump   = 0.70;     // working fluid pump efficiency
    double rho_wf     = 30.0;     // R134a vapor density [kg/m^3]
    double P_cond     = 0.50;     // condenser pressure [MPa]

    // P_inlet model: P = a0 + a1*T_h + a2*m_dot_wf  [MPa]
    double a0         = 0.10;
    double a1         = 0.016;    // MPa/^\circC
    double a2         = 3.8;      // MPa/(kg/s)
    double P_inlet_max = 1.38;    // hard pressure limit [MPa]

    // Solar collector pump
    double dp_f       = 50000.0;  // pump pressure rise [Pa] (50 kPa)
    double rho_f      = 990.0;    // water density [kg/m^3]
    double eta_f      = 0.60;     // solar pump efficiency

    // Control bounds
    double m_dot_f_min  = 0.10;   // [kg/s]
    double m_dot_f_max  = 0.50;
    double m_dot_wf_min = 0.01;
    double m_dot_wf_max = 0.08;

    // Simulation
    double T_c_nom    = 6.0;      // nominal cold source [^\circC]
    double Ts         = 30.0;     // sample time [s]
    double duration   = 5400.0;   // default duration [s]

    static PlantParams fromJson(const std::string& path);
};

struct Disturbance {
    double G      = 800.0;  // solar irradiance [W/m^2]
    double T_amb  = 20.0;   // ambient temperature [^\circC]
    double T_c    = 6.0;    // cold ocean temperature [^\circC]
};

struct ScenarioConfig {
    std::string id;
    std::string description;
    double T_h_ref      = 63.0;    // hot reservoir setpoint [^\circC]
    double T_h_init     = 20.0;    // initial T_h [^\circC]
    double T_coll_init  = 20.0;    // initial T_coll [^\circC]
    double duration_s   = 5400.0;

    // Irradiance profile
    std::string g_type  = "constant";
    double G_0          = 800.0;
    double G_1          = 0.0;
    double t_G_step     = -1.0;
    double G_amp        = 0.0;
    double G_freq       = 0.0;

    // T_h_ref profile (for step scenarios)
    std::string T_h_ref_type = "constant";
    double T_h_ref_1         = 63.0;
    double t_T_h_step        = -1.0;

    // Disturbances
    double T_amb_0      = 20.0;
    double T_c_0        = 6.0;

    double GAt(double t) const;
    double T_h_refAt(double t) const;
    Disturbance distAt(double t) const;

    static ScenarioConfig fromJson(const std::string& path);
};

struct OrcState {
    double P_inlet      = 0.0;  // expander inlet pressure [MPa]
    double dh_exp       = 0.0;  // specific expander work [J/kg]
    double W_gross      = 0.0;  // gross power [W]
    double W_pump_wf    = 0.0;  // WF pump work [W]
    double W_solar_pump = 0.0;  // solar pump work [W]
    double W_net        = 0.0;  // net power [W]
    double Q_evap       = 0.0;  // evaporator heat input [W]
    double eta_th       = 0.0;  // thermal efficiency
    double delta_T_super = 0.0; // superheating degree [^\circC]
};

class SotecPlant {
public:
    explicit SotecPlant(const PlantParams& p);

    // Advance one Ts step (forward Euler).
    void step(double m_dot_f, double m_dot_wf, const Disturbance& d);

    const Eigen::Vector2d& state() const { return x_; }
    double T_h()    const { return x_(0); }
    double T_coll() const { return x_(1); }

    // Compute ORC algebraic outputs at current state and given inputs
    OrcState computeOrc(double m_dot_f, double m_dot_wf, const Disturbance& d) const;

    // Safety: max m_dot_wf that keeps P_inlet <= P_inlet_max
    double m_dot_wf_max_safe(double T_h) const;

    void reset(double T_h_init = 20.0, double T_coll_init = 20.0);
    const PlantParams& params() const { return p_; }

private:
    PlantParams     p_;
    Eigen::Vector2d x_;  // [T_h, T_coll]

    double Q_evap_from_state(double T_h, double m_dot_wf, double T_c) const;
};

} // namespace sotec
