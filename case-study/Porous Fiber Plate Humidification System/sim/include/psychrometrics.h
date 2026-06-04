#pragma once
#include <algorithm>
#include <cmath>

// Psychrometric helpers for the porous-plate humidification case study.
// All temperatures are in Kelvin unless the function name ends in _C.
// Reference: standard psychrometric relations + Antoine equation from paper.

namespace humid {
namespace psychro {

static constexpr double kPatm  = 101325.0;  // atmospheric pressure [Pa]
static constexpr double kRw    = 461.89;    // specific gas constant for water vapour [J/(kg.K)]
static constexpr double kD0    = 2.2e-5;    // water-vapour diffusivity at T0 [m^2/s]
static constexpr double kT0    = 273.0;     // reference temperature for diffusivity [K]
static constexpr double kM_w   = 0.018015;  // molar mass of water [kg/mol]
static constexpr double kCp_a  = 1006.0;    // specific heat of dry air [J/(kg.K)]
static constexpr double kLambda_v = 2501000.0; // latent heat of vaporisation at 0^\circC [J/kg]

// Saturation vapour pressure [Pa] (Antoine/Magnus form from paper, Eq. implicit).
// Valid for -20^\circC to 60^\circC.
inline double Psat(double T_K)
{
    double T_C = T_K - 273.15;
    return 610.78 * std::exp(17.2694 * T_C / (T_C + 237.29));
}

// Specific humidity from partial pressure [kg water / kg dry air].
inline double omega_from_Pp(double Pp_Pa)
{
    return 0.622 * Pp_Pa / std::max(kPatm - Pp_Pa, 1.0);
}

// Relative humidity from specific humidity and temperature.
inline double phi_from_omega(double omega_gkg, double T_K)
{
    double ps = Psat(T_K);
    // omega in g/kg -> divide by 1000 for SI
    double x = omega_gkg / 1000.0;
    return x * kPatm / ((0.622 + x) * ps);
}

// Specific humidity [g/kg] from relative humidity and temperature.
inline double omega_gkg_from_phi(double phi, double T_K)
{
    double Pa = phi * Psat(T_K);
    return 1000.0 * 0.622 * Pa / (kPatm - Pa);
}

// Wet-bulb temperature [K] via iterative Newton solution (5 iterations).
// Uses the psychrometric equation:  omega_s(Twb) = omega_a + (Cp_a/Lambda_v)*(Ta - Twb)
inline double wetBulb(double Ta_K, double phi_a)
{
    double Pa      = phi_a * Psat(Ta_K);
    double omega_a = omega_from_Pp(Pa);   // kg/kg

    // Initial guess: approximate Twb by linear offset from Ta
    double Twb = Ta_K - (1.0 - phi_a) * 12.0;
    Twb = std::max(Twb, 273.15);

    const double cp_lambda = kCp_a / kLambda_v;  // approx = 4.02e-4

    for (int i = 0; i < 8; ++i) {
        double ps_wb    = Psat(Twb);
        double omega_s  = omega_from_Pp(ps_wb);   // kg/kg

        double F  = omega_s - omega_a - cp_lambda * (Ta_K - Twb);

        // dF/dTwb  = d(omega_s)/dTwb + cp_lambda
        double dps_dT = ps_wb * 17.2694 * 237.29
                        / ((Twb - 273.15 + 237.29) * (Twb - 273.15 + 237.29));
        double domegas_dT = 0.622 * kPatm / ((kPatm - ps_wb) * (kPatm - ps_wb)) * dps_dT;
        double dF_dT  = domegas_dT + cp_lambda;

        double step = -F / dF_dT;
        Twb += step;
        Twb  = std::max(Twb, 273.15);
        if (std::abs(step) < 1e-4) break;
    }
    return Twb;
}

// Air kinematic viscosity [m^2/s] (linear fit, valid 250-340 K).
inline double nu_air(double T_K)
{
    return 1.328e-5 + 9.6e-8 * (T_K - 293.15);
}

// Air thermal conductivity [W/(m.K)] (linear fit).
inline double lambda_air(double T_K)
{
    return 0.02442 + 7.2e-5 * (T_K - 293.15);
}

// Air Prandtl number (nearly constant for dry air 20-60^\circC).
inline double Pr_air(double /*T_K*/) { return 0.707; }

// Water-vapour diffusivity in air [m^2/s] at mean temperature Tm_K.
inline double D_wv(double Tm_K)
{
    return kD0 * std::pow(Tm_K / kT0, 1.5);
}

} // namespace psychro
} // namespace humid
