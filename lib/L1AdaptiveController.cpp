#include "L1AdaptiveController.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ctrl {

L1AdaptiveController::L1AdaptiveController(const Params& p, double Ts)
    : p_(p), Ts_(Ts)
{
    if (std::abs(p_.a_m) >= 1.0)
        throw std::invalid_argument("L1AdaptiveController: |a_m| must be < 1");
    if (p_.b_m == 0.0)
        throw std::invalid_argument("L1AdaptiveController: b_m must be non-zero");
    if (p_.Gamma <= 0.0)
        throw std::invalid_argument("L1AdaptiveController: Gamma must be positive");
    if (p_.omega_c <= 0.0)
        throw std::invalid_argument("L1AdaptiveController: omega_c must be positive");

    // Discrete Lyapunov: a_m^2 * P - P + Q = 0  =>  P = Q / (1 - a_m^2)
    P_ = p_.Q_lyap / (1.0 - p_.a_m * p_.a_m);

    // Low-pass filter: first-order IIR, C(z) = (1-a_lp)/(z - a_lp)
    a_lp_ = std::exp(-p_.omega_c * Ts_);
    b_lp_ = 1.0 - a_lp_;
}

double L1AdaptiveController::compute(double y_plant)
{
    // Step 1: prediction error
    pred_err_         = x_hat_ - y_plant;

    // Step 2: adaptation law (gradient descent on Lyapunov function)
    // sigma^[k+1] = sigma^[k] - Gamma.Ts.(ẽ[k] . b_m . P)
    sigma_hat_        -= p_.Gamma * Ts_ * (pred_err_ * p_.b_m * P_);
    sigma_hat_         = std::clamp(sigma_hat_, -p_.sigma_max, p_.sigma_max);

    // Step 3: desired input to the LP filter  eta = -sigma^
    //         full signal = k_g*r + eta
    const double eta     = -sigma_hat_;
    const double signal  = p_.k_g * r_ + eta;

    // Step 4: LP filter  v[k+1] = a_lp * v[k] + b_lp * signal
    v_lp_   = a_lp_ * v_lp_ + b_lp_ * signal;
    double u = std::clamp(v_lp_, p_.uMin, p_.uMax);

    // Step 5: advance state predictor
    // x^[k+1] = a_m * x^[k] + b_m * (u[k] + sigma^[k])
    x_hat_  = p_.a_m * x_hat_ + p_.b_m * (u + sigma_hat_);

    notifyObserver(u, y_plant);
    return u;
}

void L1AdaptiveController::reset()
{
    x_hat_     = 0.0;
    sigma_hat_ = 0.0;
    v_lp_      = 0.0;
    pred_err_  = 0.0;
}

} // namespace ctrl
