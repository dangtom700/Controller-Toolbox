#pragma once
#include "IController.h"
#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
#include <string>

namespace ctrl {

/**
 * @brief L1 Adaptive Controller (SISO).
 *
 * Hovakimyan & Cao, *L1 Adaptive Control Theory*, AIAA 2010.
 *
 * The key architectural difference from MRAC: a **low-pass filter C(s)**
 * is inserted in the control channel.  This separates the adaptation
 * time-scale (which can be arbitrarily fast) from the control bandwidth,
 * guaranteeing bounded transient performance regardless of the adaptation gain Gamma.
 *
 * **Architecture (discrete-time, 1st-order plant model):**
 * @code
 *   State predictor:  x^[k+1] = a_m . x^[k] + b_m . (u[k] + sigma^[k])
 *   Adaptation law:   sigma^[k]   += Gamma . Ts . (-ẽ[k] . b_m . P)
 *                     sigma^       clamped to [-sigma_max, sigma_max]
 *   Control law:      eta[k]    = -sigma^[k]
 *                     u[k]    = C(z) . (k_g . r[k] + eta[k])
 * @endcode
 *
 * Where ẽ[k] = x^[k] - x[k] (prediction error) and P solves the Lyapunov
 * equation  a_m^2 . P - P = -Q  (discrete-time, Q > 0, a_m stable).
 * C(z) is a first-order low-pass filter:
 * @code
 *   C(z) = (1 - a_lp) / (z - a_lp)    a_lp = exp(-omega_c . Ts)
 * @endcode
 *
 * **Comparison with MRACController:**
 *  - MRAC: no LP filter; adaptation directly modifies u; transient not bounded.
 *  - L1:   LP filter guarantees |u(t) - u_perf(t)| < epsilon regardless of Gamma.
 *
 * **Usage:**
 * @code
 *   ctrl::L1AdaptiveController::Params p;
 *   p.a_m    = 0.85;     // reference model pole (discrete)
 *   p.b_m    = 0.15;     // reference model input gain (DC gain = 1 if b_m=1-a_m)
 *   p.Gamma  = 100.0;    // adaptation gain (can be large with L1)
 *   p.omega_c = 0.5;     // LP filter bandwidth [rad/s]  (determines robustness margin)
 *   p.sigma_max = 10.0;
 *   ctrl::L1AdaptiveController ctrl(p, Ts);
 *
 *   ctrl.setReference(r);
 *   double u = ctrl.compute(y_plant);   // absolute plant output, NOT error
 * @endcode
 *
 * @note compute() receives the absolute plant output (same convention as MRACController).
 *       Call setReference(r) each step before compute(y).
 */
class L1AdaptiveController : public IController {
public:
    struct Params {
        double a_m      = 0.85;    ///< Reference model pole (|a_m| < 1 required)
        double b_m      = 0.15;    ///< Reference model input gain (b_m = 1 - a_m for unity DC)
        double k_g      = 1.0;     ///< Feedforward gain ( = 1/DC_gain_ref if DC tracking needed )
        double Gamma    = 50.0;    ///< Adaptation gain (large is OK: LP filter provides robustness)
        double omega_c  = 2.0;     ///< LP filter bandwidth [rad/s] (robustness-bandwidth tradeoff)
        double sigma_max= 20.0;    ///< Max magnitude of estimated disturbance sigma^
        double Q_lyap   = 1.0;     ///< Discrete Lyapunov weight for P computation (Q > 0)
        double uMin     = -1e9;    ///< Output lower bound
        double uMax     =  1e9;    ///< Output upper bound
    };

    L1AdaptiveController(const Params& p, double Ts);

    // ---- Primary interface -------------------------------------------------
    /** @brief Store the setpoint for the next compute() call. */
    void   setReference(double r) noexcept { r_ = r; }

    /**
     * @brief Compute control action.
     * @param y_plant Absolute plant output (NOT the tracking error).
     *        Same convention as MRACController: pass y directly, not r-y.
     *        Call setReference(r) before this.
     * @return Control input u.
     */
    double compute(double y_plant) override;

    void        reset()             override;
    double      sampleTime() const  override { return Ts_; }
    std::string name()        const override { return "L1Adaptive"; }

    // ---- Diagnostics -------------------------------------------------------
    double estimatedDisturbance() const noexcept { return sigma_hat_; }
    double predictionError()      const noexcept { return pred_err_;  }

private:
    Params p_;
    double Ts_;

    // Reference model: x_m[k+1] = a_m * x_m[k] + b_m * (u[k] + sigma_hat[k])
    double x_hat_   = 0.0;   ///< State predictor

    // Adaptation
    double sigma_hat_ = 0.0; ///< Estimated unknown disturbance/uncertainty
    double P_;               ///< Lyapunov solution: a_m^2*P - P = -Q  -> P = Q/(1-a_m^2)

    // Low-pass filter state: C(z) = b_lp / (z - a_lp)
    double a_lp_;            ///< LP pole  = exp(-omega_c * Ts)
    double b_lp_;            ///< LP gain  = 1 - a_lp
    double v_lp_   = 0.0;   ///< LP filter state

    double r_        = 0.0;  ///< Stored reference
    double pred_err_ = 0.0;  ///< Last prediction error |x_hat - y|

    bool use_compute_y_ = false; // flag to distinguish compute(y) vs compute(error) calls
};

} // namespace ctrl
