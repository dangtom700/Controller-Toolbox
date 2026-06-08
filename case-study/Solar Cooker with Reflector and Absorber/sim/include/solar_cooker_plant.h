#pragma once
#include <Eigen/Dense>
#include <string>
#include <vector>

// Solar cooker with TBPR + PCM absorber plant model.
//
// Reference: Tawfik et al. (2026) Energy 344, 140064.
//
// 2 states: x = [T_abs, T_pot]   [^\circC]
// Input:    u = f_shade [0, 1]   (shade fraction; 0 = fully open)
// Disturbances: G [W/m^2], T_amb [^\circC], V_wind [m/s]
//
// PCM handled via effective heat capacity method:
//   C_eff = m_abs*cp_abs + m_pcm*lambda_pcm/delta_T   (inside melting band)
//   C_eff = m_abs*cp_abs                               (outside melting band)
//
// Integration: classical RK4 with N_SUBSTEPS=10 sub-steps per Ts (3 s each).

namespace cooker {

static constexpr int N_SUBSTEPS = 10;

struct PlantParams {
    // Box + TBPR optics
    double A_box     = 0.36;   // box aperture area [m^2]
    double A_abs     = 0.36;   // absorber area [m^2]
    double A_c       = 0.04;   // absorber-to-pot contact area [m^2]
    double A_pot     = 0.02;   // pot surface area [m^2]
    double eta_box   = 0.60;   // box optical efficiency
    double eta_TBPR  = 0.18;   // TBPR additional efficiency

    // Absorber thermal mass
    double m_abs     = 2.0;    // absorber mass [kg]
    double cp_abs    = 900.0;  // absorber specific heat [J/(kg K)]

    // Cooking pot thermal mass
    double m_pot     = 2.0;    // pot mass [kg]
    double cp_pot    = 4186.0; // pot specific heat [J/(kg K)]

    // PCM (paraffin wax in absorber)
    double m_pcm         = 0.5;     // PCM mass [kg]
    double lambda_pcm    = 230.0e3; // PCM latent heat [J/kg]
    double T_melt        = 55.0;    // PCM melt midpoint [^\circC]
    double delta_T_pcm   = 4.0;     // PCM melting band width [^\circC]

    // Heat transfer coefficients
    double h_abs_amb = 8.0;    // absorber-to-ambient convection [W/(m^2 K)]
    double h_abs_pot = 60.0;   // absorber-to-pot coupling [W/(m^2 K)]
    double U_loss    = 8.0;    // pot overall loss coefficient [W/(m^2 K)]
    double eps_abs   = 0.95;   // absorber emissivity
    double sigma     = 5.67e-8;// Stefan-Boltzmann [W/(m^2 K^4)]

    // Safety and integration
    double T_abs_max = 280.0;  // absorber hard limit [^\circC]
    double Ts        = 30.0;   // sample time [s]
    double duration  = 5400.0; // default scenario duration [s]

    static PlantParams fromJson(const std::string& path);
};

struct Disturbance {
    double G      = 800.0;  // solar irradiance [W/m^2]
    double T_amb  = 30.0;   // ambient temperature [^\circC]
    double V_wind = 1.0;    // wind speed [m/s]
};

struct ScenarioConfig {
    std::string id;
    std::string description;
    double T_ref        = 95.0;   // cooking pot target [^\circC]
    double T_abs_init   = 30.0;   // initial absorber temperature [^\circC]
    double T_pot_init   = 30.0;   // initial pot temperature [^\circC]
    double duration_s   = 5400.0;

    // Irradiance profile
    std::string g_type  = "constant";  // "constant", "step", "sinusoidal"
    double G_0          = 800.0;
    double G_1          = 0.0;         // used if g_type == "step" after t_G_step
    double t_G_step     = -1.0;        // time of G step [s] (-1 = no step)
    double G_amp        = 0.0;         // sinusoidal amplitude [W/m^2]
    double G_freq       = 0.0;         // sinusoidal frequency [rad/s]

    // Ambient temperature
    double T_amb_0      = 30.0;

    // Wind speed profile
    std::string v_type  = "constant";  // "constant", "step"
    double V_wind_0     = 1.0;
    double V_wind_1     = 1.0;
    double t_wind_step  = -1.0;

    double GAt(double t) const;
    double VWindAt(double t) const;
    Disturbance distAt(double t) const;

    static ScenarioConfig fromJson(const std::string& path);
};

class SolarCookerPlant {
public:
    explicit SolarCookerPlant(const PlantParams& p);

    // Advance one Ts step (internal RK4 sub-stepping).
    // Returns T_abs AFTER clamping to T_abs_max.
    void step(double f_shade, const Disturbance& d);

    const Eigen::Vector2d& state() const { return x_; }
    double T_abs() const { return x_(0); }
    double T_pot() const { return x_(1); }

    // phi_pcm = PCM liquid fraction [0, 1]
    double phiPCM() const;
    // Q_absorbed = instantaneous solar heat absorbed by absorber [W]
    double Q_absorbed(double f_shade, double G) const;

    void reset(double T_abs_init = 30.0, double T_pot_init = 30.0);
    const PlantParams& params() const { return p_; }

private:
    PlantParams     p_;
    Eigen::Vector2d x_;  // [T_abs, T_pot]

    // ODE RHS: dx/dt = f(x, u, d)
    Eigen::Vector2d ode(const Eigen::Vector2d& x,
                        double f_shade,
                        const Disturbance& d) const;

    // Effective absorber heat capacity (includes PCM latent heat if melting)
    double C_eff(double T_abs) const;
};

} // namespace cooker
