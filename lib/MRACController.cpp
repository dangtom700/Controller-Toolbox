#include "MRACController.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ctrl
{

MRACController::MRACController(const MRACParams &params, double Ts)
    : params_(params), Ts_(Ts), r_(0.0), e_m_(0.0)
{
    if (std::abs(params_.a_m) >= 1.0)
        throw std::invalid_argument("MRACController: |a_m| must be < 1 (stable reference model).");
    if (Ts_ <= 0.0)
        throw std::invalid_argument("MRACController: Ts must be positive.");
    if (params_.theta_max <= 0.0)
        throw std::invalid_argument("MRACController: theta_max must be positive.");
    if (params_.uMin >= params_.uMax)
        throw std::invalid_argument("MRACController: uMin must be < uMax.");

    // Default theta_r0 = b_m (nominal feedforward for unity-gain plant)
    theta_r_ = (params_.theta_r0 != 0.0) ? params_.theta_r0 : params_.b_m;
    theta_y_ = params_.theta_y0;
    y_m_     = 0.0;
}

double MRACController::compute(double y_plant)
{
    if (!std::isfinite(y_plant)) return 0.0;
    // 1. Model tracking error: e_m[k] = y[k] - y_m[k]
    e_m_ = y_plant - y_m_;

    // 2. Control law: u[k] = theta_r * r + theta_y * y
    double u = theta_r_ * r_ + theta_y_ * y_plant;
    u = std::clamp(u, params_.uMin, params_.uMax);

    // 3. Lyapunov adaptation with sigma-modification (discrete gradient descent + damping)
    //    theta[k+1] = theta[k] - Ts * (gamma * e_m * phi + sigma * theta[k])
    theta_r_ -= Ts_ * (params_.gamma_r * e_m_ * r_        + params_.sigma * theta_r_);
    theta_y_ -= Ts_ * (params_.gamma_y * e_m_ * y_plant   + params_.sigma * theta_y_);

    // 4. Parameter projection: clamp ||theta||2 <= theta_max
    const double theta_norm = std::sqrt(theta_r_ * theta_r_ + theta_y_ * theta_y_);
    if (theta_norm > params_.theta_max)
    {
        const double scale = params_.theta_max / theta_norm;
        theta_r_ *= scale;
        theta_y_ *= scale;
    }

    // 5. Advance reference model: y_m[k+1] = a_m * y_m[k] + b_m * r[k]
    y_m_ = params_.a_m * y_m_ + params_.b_m * r_;

    return u;
}

void MRACController::reset()
{
    theta_r_ = (params_.theta_r0 != 0.0) ? params_.theta_r0 : params_.b_m;
    theta_y_ = params_.theta_y0;
    y_m_     = 0.0;
    r_       = 0.0;
    e_m_     = 0.0;
}

void MRACController::setParams(const MRACParams &p)
{
    params_  = p;
    theta_r_ = (p.theta_r0 != 0.0) ? p.theta_r0 : p.b_m;
    theta_y_ = p.theta_y0;
}

} // namespace ctrl
