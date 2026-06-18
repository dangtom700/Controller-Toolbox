#pragma once
#include <Eigen/Dense>
#include "IController.h"

/**
 * @file MRACController.h
 * @brief Discrete-time Model Reference Adaptive Control (MRAC) - SISO, relative degree 1.
 *
 * Forces the closed-loop output to track the output of a user-specified reference model M(z):
 * @code
 *   y_m[k+1] = a_m . y_m[k] + b_m . r[k]      (first-order discrete reference model)
 * @endcode
 *
 * **Control law (two-parameter MRAC):**
 * @code
 *   u[k] = theta_r[k] . r[k]  +  theta_y[k] . y[k]
 * @endcode
 *
 * **Lyapunov-based discrete adaptation with sigma-modification (Ioannou-Kokotovic 1983):**
 * @code
 *   theta_r[k+1] = theta_r[k] - Ts.( gamma_r . e_m[k] . r[k]  + sigma . theta_r[k] )
 *   theta_y[k+1] = theta_y[k] - Ts.( gamma_y . e_m[k] . y[k]  + sigma . theta_y[k] )
 *   e_m[k] = y[k] - y_m[k]   (tracking error, plant minus reference model)
 * @endcode
 *
 * sigma-modification prevents parameter drift in the presence of noise or unmodelled dynamics
 * by adding a damping term -sigma.theta to the gradient. Set sigma = 0 to disable.
 *
 * **Parameter projection** keeps ||[theta_r, theta_y]||2 <= theta_max, ensuring bounded parameters even
 * when the reference signal lacks Persistent Excitation.
 *
 * @par Feasibility conditions
 *  - Plant must be **minimum phase** (all zeros strictly inside the unit circle).
 *  - Plant input gain b_p must have **known sign** (same sign as gamma_r, gamma_y).
 *    For a positive-gain plant, use gamma_r, gamma_y > 0 (default).
 *    For a negative-gain plant, negate both (gamma_r < 0, gamma_y < 0).
 *  - Reference signal r[k] must be **persistently exciting** for parameter convergence
 *    (constant r -> tracking error converges, but theta may not reach true values).
 *  - |a_m| < 1 is required for the reference model to be stable.
 *
 * @par Usage (ADRC convention: compute takes plant output y, not error)
 * @code
 *   ctrl::MRACParams mp;
 *   mp.a_m = 0.5;           // desired closed-loop pole
 *   mp.b_m = 0.5;           // = 1 - a_m for unity DC gain
 *   mp.gamma_r = 2.0;       // adaptation rate (tune: larger = faster, may oscillate)
 *   mp.gamma_y = 2.0;
 *   mp.sigma   = 0.005;     // sigma-modification (small positive value)
 *   mp.theta_max = 20.0;
 *
 *   ctrl::MRACController mrac(mp, Ts);
 *
 *   for (int k = 0; k < N; ++k) {
 *       mrac.setReference(r);
 *       double u = mrac.compute(y);   // takes plant output, returns control
 *       // apply u to plant, get next y
 *   }
 * @endcode
 *
 * @see Astrom & Wittenmark, "Adaptive Control", 2nd ed. (1995), Ch. 4.
 * @see Ioannou & Sun, "Robust Adaptive Control" (1996), Ch. 5.
 */

namespace ctrl
{

/** @brief Parameters for MRACController. */
struct MRACParams
{
    double a_m       = 0.5;   ///< Reference model pole (|a_m| < 1 for stability). y_m[k+1]=a_m*y_m+b_m*r.
    double b_m       = 0.5;   ///< Reference model gain (set b_m=1-a_m for DC gain 1).
    double gamma_r   = 1.0;   ///< Adaptation rate for theta_r > 0 (positive for positive-gain plant).
    double gamma_y   = 1.0;   ///< Adaptation rate for theta_y > 0.
    double sigma     = 0.005; ///< sigma-modification gain >= 0 (0 = off; 0.005-0.05 typical).
    double theta_max = 50.0;  ///< Parameter projection bound: ||[theta_r, theta_y]||2 <= theta_max.
    double theta_r0  = 0.0;   ///< Initial theta_r (0 -> library sets b_m at construction).
    double theta_y0  = 0.0;   ///< Initial theta_y.
    double uMin      = -1e6;  ///< Lower output saturation limit.
    double uMax      =  1e6;  ///< Upper output saturation limit.
};

/**
 * @brief Discrete-time MRAC controller - SISO, first-order reference model.
 *
 * Implements `IController` with the ADRC output convention:
 * `compute(y_plant)` takes the plant output (not the error).
 * Call `setReference(r)` once per step before `compute()`.
 */
class MRACController : public IController
{
public:
    /**
     * @brief Construct with parameters and sample time.
     * @param params  MRAC tuning parameters.
     * @param Ts      Sample time [s].
     * @throws std::invalid_argument if |a_m| >= 1, Ts <= 0, gamma_r or gamma_y == 0,
     *   theta_max <= 0, or uMin >= uMax.
     */
    MRACController(const MRACParams &params, double Ts);

    /**
     * @brief Compute one control step.
     *
     * Execution order:
     *  1. e_m = y - y_m[k]                       (model tracking error)
     *  2. u = clamp(theta_r.r + theta_y.y, uMin, uMax)  (control law)
     *  3. theta_r, theta_y updated via gradient + sigma-mod  (adaptation)
     *  4. ||theta|| projected if > theta_max
     *  5. y_m[k+1] = a_m.y_m + b_m.r            (advance reference model)
     *
     * @param y_plant Current plant output y[k].
     * @return        Control signal u[k].
     */
    double compute(double y_plant) override;

    /** @brief Reset theta to initial values, y_m to 0, e_m to 0. */
    void reset() override;

    /** @brief Sample time Ts [s]. */
    double sampleTime() const override { return Ts_; }

    /** @brief Set the current reference r[k]. Must be called before each compute(). */
    void setReference(double r) { r_ = r; }

    /** @brief Current feed-forward parameter theta_r. */
    double theta_r() const { return theta_r_; }

    /** @brief Current output-feedback parameter theta_y. */
    double theta_y() const { return theta_y_; }

    /** @brief Reference model output y_m[k] (updated inside compute()). */
    double modelOutput() const { return y_m_; }

    /** @brief Tracking error e_m[k] = y[k] - y_m[k] from the last compute() call. */
    double modelError() const { return e_m_; }

    /** @brief Current parameter set. */
    const MRACParams &params() const { return params_; }

    /** @brief Update parameters at runtime (resets theta to new theta_r0/theta_y0 if changed). */
    void setParams(const MRACParams &p);

    /** @brief Seed the reference model state y_m without triggering adaptation.
     *  Useful when the plant output is already at a known value at t=0 so the
     *  reference model starts tracking from that point rather than from 0. */
    void setYm(double y_m) { y_m_ = y_m; }

private:
    MRACParams params_;
    double     Ts_;
    double     theta_r_;
    double     theta_y_;
    double     y_m_;      ///< Reference model state y_m[k].
    double     r_;        ///< Current reference (set by setReference).
    double     e_m_;      ///< Last tracking error y - y_m.
    double     u_prev_ = 0.0; ///< Last finite compute() return value (hold-last NaN contract).
};

} // namespace ctrl
