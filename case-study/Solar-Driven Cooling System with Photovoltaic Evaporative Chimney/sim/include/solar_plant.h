#pragma once
#include <cmath>
#include <string>

// Solar-Driven Cooling System with PV Evaporative Chimney - physics plant.
//
// Source: Ruiz, Martinez, Aguilar, Lucas
//         "Analytical Modelling and Optimisation of a Solar-Driven Cooling
//          System Enhanced with a Photovoltaic Evaporative Chimney"
//         Applied Thermal Engineering 245 (2024) 122878
//
// Architecture (Fig. 3 of the paper - three sequential algebraic sub-models):
//
//   Step 1  Evaporative area  (Poppe ODEs, Eqs. 13-18)
//               RK4 over water temperature from Tw_warm down to Tw_cool.
//               Coupled to chiller EER quadratic (Eq. 20) via energy balance.
//               Outputs: Tw1, T_ai, m_dot_a, Q_cond, W_comp, EER.
//
//   Step 2  PV convective area  (closed-form 4-layer heat balance, Eqs. 1-8)
//               Coupling input: T_ai from Step 1 (= chimney internal air temp).
//               Outputs: Tc, eta_PV, W_PV.
//
//   Step 3  Condenser hydraulic loop  (Eqs. 22-27)
//               Analytical pump / system curve intersection at VFD speed kr.
//               Output: W_pump.
//
//   Global:  EER_grid = Q_evap / (W_comp + W_pump - W_PV)          (Eq. 28)
//
// Control inputs:
//   m_dot_w  : chimney spray water flow  [kg/s]  (validity: 0.02 - 0.22)
//   kr       : pump VFD speed ratio      [-]      (0.3 - 1.0)
//
// Weather disturbances:
//   G        : solar irradiance          [W/m^2]
//   T_amb    : ambient dry-bulb          [^\circC]
//   phi_amb  : relative humidity         [-]
//   v_w      : wind speed                [m/s]

namespace solar {

// ---------------------------------------------------------------------------
// Parameter groups (loaded from config/plant_params.json via PlantParams)
// ---------------------------------------------------------------------------

struct PVParams {
    double area;        // panel area [m^2]
    double tau;         // glass transmittance [-]
    double eta_ref;     // reference efficiency [-]
    double beta_ref;    // temperature coefficient [1/^\circC]
    double T_ref;       // reference temperature [^\circC]
    double G_ref;       // reference irradiance [W/m^2]
    double T_NOCT;      // NOCT cell temperature [^\circC]
    double T_ref_NOCT;  // NOCT ambient reference [^\circC]
    double G_ref_NOCT;  // NOCT irradiance reference [W/m^2]
    double gamma;       // irradiance correction coefficient [-]
    double kg_xg;       // glass thermal conductance k/x [W/m^2/K]
    double kc_xc;       // cell  thermal conductance k/x [W/m^2/K]
    double kt_xt;       // tedlar thermal conductance k/x [W/m^2/K]
};

struct EvapParams {
    double me_a;        // Merkel correlation: Me = a*(mw/ma)^b  (Eq. 17)
    double me_b;
    double mair_a;      // air mass flow: ma = a*mw^2 + b*mw + c  (Eq. 18)
    double mair_b;
    double mair_c;
    double vi_a;        // internal air velocity: vi = a*mw^2 + b*mw + c  (Eq. 7)
    double vi_b;
    double vi_c;
    double Le;          // Lewis factor
    double cp_w;        // specific heat of water     [J/kg/K]
    double cp_ma;       // specific heat of moist air [J/kg/K]
    double h_fg;        // latent heat of vaporisation [J/kg]
    int    rk4_steps;   // Poppe ODE integration steps
};

struct ChillerParams {
    double Q_evap_kW;   // design cooling capacity [kW]
    double T_w2_evap;   // chiller evaporator outlet water temp [^\circC]
    // EER quadratic (Eq. 20): EER = a + b*Tw2e + c*Tw2e^2 + d*Tw1 + e*Tw1^2 + f*Tw2e*Tw1
    double a, b, c, d, e, f;
};

struct HydraulicsParams {
    double H0;          // nominal pump head [m]
    double Q0;          // nominal pump flow [l/s]
    double eta_p0;      // nominal pump efficiency [-]
    double sys_a;       // system curve: Hm = sys_a + sys_b*Q^2
    double sys_b;
    double rho;         // water density [kg/m^3]
    double g;           // gravitational acceleration [m/s^2]
    double kr_min;
    double kr_max;
};

struct PlantParams {
    PVParams        pv;
    EvapParams      evap;
    ChillerParams   chiller;
    HydraulicsParams hydraulics;
    double dt;          // simulation step size [s]
    double duration;    // total simulation duration [s]
    double design_Q_m3h; // design-point condenser loop flow [m^3/h]

    // Load all parameters from a JSON file (nlohmann::json).
    static PlantParams fromJson(const std::string& path);
};

// ---------------------------------------------------------------------------
// Inputs / outputs
// ---------------------------------------------------------------------------

struct WeatherInput {
    double G;       // solar irradiance [W/m^2]
    double T_amb;   // ambient dry-bulb temperature [^\circC]
    double phi_amb; // relative humidity [-]
    double v_w;     // wind speed [m/s]
};

struct ControlInput {
    double m_dot_w; // chimney spray water mass-flow [kg/s]
    double kr;      // pump VFD speed ratio [-]
};

struct PlantOutput {
    // Step 1 - Evaporative zone + chiller
    double Tw1_C;       // warm water temp (chiller condenser in) [^\circC]
    double Tw2_C;       // cool water temp (chiller condenser out) [^\circC]
    double T_ai_C;      // air exit temperature from evap zone [^\circC]
    double ma_dot;      // air mass flow rate [kg/s]
    double Q_cond_kW;   // condenser heat rejection [kW]
    double W_comp_kW;   // compressor power [kW]
    double EER;         // energy efficiency ratio [-]
    double Me_calc;     // calculated Merkel number
    double Me_target;   // target Merkel number (correlation)
    // Step 2 - PV panel
    double Tc_C;        // cell temperature [^\circC]
    double eta_PV;      // PV efficiency [-]
    double W_PV_W;      // PV electrical output [W]
    double vi_ms;       // chimney internal air velocity [m/s]
    // Step 3 - Hydraulics
    double Q_op_ls;     // pump operating flow [l/s]
    double Hm_m;        // pump head [m]
    double W_pump_W;    // pump power [W]
    double eta_pump;    // pump efficiency [-]
    // Global
    double EER_grid;    // grid EER (Eq. 28)
    double W_net_kW;    // net grid consumption [kW]
};

// ---------------------------------------------------------------------------
// Plant facade
// ---------------------------------------------------------------------------

class SolarCoolingPlant {
public:
    explicit SolarCoolingPlant(const PlantParams& params);

    static SolarCoolingPlant fromJson(const std::string& path);

    // Compute one steady-state operating point.
    // All sub-models are algebraic - no internal time-integration state.
    PlantOutput step(const WeatherInput& w, const ControlInput& u);

    const PlantParams& params() const { return p_; }
    const PlantOutput& lastOutput() const { return last_; }

private:
    PlantParams p_;
    PlantOutput last_{};
};

} // namespace solar
