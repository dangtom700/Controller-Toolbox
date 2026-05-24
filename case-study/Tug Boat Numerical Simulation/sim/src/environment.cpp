#include "environment.h"
#include <cmath>
#include <random>

using namespace Eigen;

namespace tug {

Environment::Environment(const PlantParameters& p,
                         const EnvConditions& cond,
                         uint32_t seed)
    : pp_(p), cond_(cond)
{
    // Use plant Hs/Tp if not overridden in conditions
    if (cond_.Hs <= 0.0) cond_.Hs = p.Hs;
    if (cond_.Tp <= 0.0) cond_.Tp = p.Tp;
    initWaves(seed);
}

void Environment::initWaves(uint32_t seed)
{
    // JONSWAP peak frequency
    const double omega_p = 2.0 * M_PI / cond_.Tp;
    const double g       = 9.81;
    const double alpha_j = 0.0081;  // Phillips constant

    // Two components bracketing the spectral peak
    const double omega[2] = { omega_p * 0.90, omega_p * 1.10 };

    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> phase_dist(0.0, 2.0 * M_PI);

    for (int j = 0; j < 2; ++j) {
        double w = omega[j];
        double sigma = (w <= omega_p) ? 0.07 : 0.09;
        double r     = std::exp(-std::pow(w - omega_p, 2) /
                                (2.0 * sigma * sigma * omega_p * omega_p));
        // JONSWAP spectrum value
        double S = alpha_j * g * g / std::pow(w, 5)
                   * std::exp(-1.25 * std::pow(omega_p / w, 4))
                   * std::pow(pp_.gamma_j, r);
        double dw = omega_p * 0.20; // bandwidth of each component
        double amp = std::sqrt(2.0 * S * dw);

        waves_[j] = {amp, w, phase_dist(rng)};
    }
}

Vector3d Environment::windLoad(const Vector3d& nu) const
{
    if (cond_.wind_speed < 1e-6) return Vector3d::Zero();

    // Apparent wind velocity components in body frame
    // World-frame wind velocity: U_w = wind_speed from bearing wind_bearing
    double Vwx =  cond_.wind_speed * std::cos(cond_.wind_bearing); // world +x component
    double Vwy =  cond_.wind_speed * std::sin(cond_.wind_bearing);

    // Relative wind (body frame) — subtract vessel velocity u,v
    double Vrel_u = Vwx - nu(0); // surge relative
    double Vrel_v = Vwy - nu(1); // sway  relative
    double U_wr = std::sqrt(Vrel_u * Vrel_u + Vrel_v * Vrel_v);
    if (U_wr < 1e-6) return Vector3d::Zero();

    double gamma_wr = std::atan2(Vrel_v, Vrel_u); // angle of relative wind in body frame

    double q = 0.5 * pp_.rho_air * U_wr * U_wr;

    double Cwx = -pp_.Cwx_max * std::cos(gamma_wr);
    double Cwy =  pp_.Cwy_max * std::sin(gamma_wr);
    double Cwp =  pp_.Cwpsi_max * std::sin(2.0 * gamma_wr);

    return Vector3d(
        q * pp_.frontal_area * Cwx,
        q * pp_.lateral_area * Cwy,
        q * pp_.lateral_area * pp_.length * Cwp);
}

Vector3d Environment::currentLoad(const Vector3d& nu) const
{
    if (cond_.current_speed < 1e-6) return Vector3d::Zero();

    // Current velocity components in world frame
    double Vcx = cond_.current_speed * std::cos(cond_.current_bearing);
    double Vcy = cond_.current_speed * std::sin(cond_.current_bearing);

    // Relative current in body frame
    double Vrel_u = Vcx - nu(0);
    double Vrel_v = Vcy - nu(1);
    double U_cr = std::sqrt(Vrel_u * Vrel_u + Vrel_v * Vrel_v);
    if (U_cr < 1e-6) return Vector3d::Zero();

    double gamma_cr = std::atan2(Vrel_v, Vrel_u);

    double q = 0.5 * pp_.rho_water * U_cr * U_cr;

    // Underwater frontal/lateral areas
    double A_F = pp_.beam  * pp_.draft;
    double A_L = pp_.length * pp_.draft;

    double Ccx = -pp_.Cu   * std::cos(gamma_cr);
    double Ccy =  pp_.Cv   * std::sin(gamma_cr);
    double Ccp =  pp_.Cpsi * std::sin(2.0 * gamma_cr);

    return Vector3d(
        q * A_F * Ccx,
        q * A_L * Ccy,
        q * A_L * pp_.length * Ccp);
}

Vector3d Environment::waveDrift(double t) const
{
    double eta_w = 0.0;
    for (auto& wc : waves_)
        eta_w += wc.amplitude * std::cos(wc.omega * t + wc.phase);

    double env_sq = eta_w * eta_w;  // second-order (drift is proportional to amplitude^2)

    return Vector3d(
        pp_.drift_x   * env_sq,
        pp_.drift_y   * env_sq,
        pp_.drift_psi * env_sq);
}

Vector3d Environment::compute(double t,
                               const Vector3d& /*eta*/,
                               const Vector3d& nu) const
{
    return windLoad(nu) + currentLoad(nu) + waveDrift(t);
}

} // namespace tug
