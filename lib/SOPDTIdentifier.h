#pragma once
#include "DiscretePID.h"
#include <functional>
#include <vector>
#include <stdexcept>

/**
 * @file SOPDTIdentifier.h
 * @brief Second-Order Plus Dead-Time (SOPDT) step-response identification.
 *
 * Fits the continuous-time model:
 * @code
 *   G(s) = K * exp(-theta * s) / ((tau1 * s + 1)(tau2 * s + 1))
 * @endcode
 * to an open-loop step-response data set.  Supports two identification methods
 * and an IMC-PID tuning rule.
 *
 * **Step response (overdamped, tau1 >= tau2 > 0):**
 * @code
 *   y(t) = K * u_step * [1 - (tau1*exp(-(t-theta)/tau1) - tau2*exp(-(t-theta)/tau2)) / (tau1 - tau2)]
 * @endcode
 * For critically damped (tau1 = tau2 = tau):
 * @code
 *   y(t) = K * u_step * [1 - (1 + (t-theta)/tau) * exp(-(t-theta)/tau)]
 * @endcode
 *
 * **Two identification methods:**
 *
 * 1. **Graphical (tangent + percentage-crossing):**
 *    - theta from ZN tangent intercept with y = y_initial (same as FOPDT).
 *    - tau_sum = tau1 + tau2 from 63.2% crossing.
 *    - Ratio tau1/tau2 estimated from the 28.3% crossing time relative to tau_sum.
 *    - Fast; good starting point for the optimization method.
 *
 * 2. **Optimization (least-squares):**
 *    - Outer golden-section on theta, middle on tau1, inner on tau2, minimising SSE.
 *    - Enforces tau1 >= tau2 so the model is uniquely ordered.
 *    - More accurate for noisy data or asymmetric responses.
 *
 * **IMC-PID tuning rule for SOPDT (Rivera 1986 extended):**
 * @code
 *   tau_eq = tau1 + tau2
 *   Kp  = tau_eq / (K * (lambdaC + theta / 2))
 *   Ti  = tau_eq
 *   Td  = tau1 * tau2 / (tau1 + tau2)  [set to 0 for PI mode]
 * @endcode
 *
 * **Usage:**
 * @code
 *   ctrl::SOPDTIdentifier id(t_vec, y_vec, step_mag);
 *   auto m = id.identify();           // graphical by default
 *   auto mo = id.identify(ctrl::SOPDTMethod::Optimization);
 *   auto pp = ctrl::SOPDTIdentifier::imcTuning(mo, 2.0 * mo.theta, Ts);
 *   ctrl::DiscretePID pid(pp, Ts);
 * @endcode
 *
 * @see FOPDTIdentifier for first-order identification.
 * @see pdc-master/Module_07/1_Topic/FirstOrderOptimization_*.py (reference data).
 * @see Rivera, Morari & Skogestad, "Internal Model Control", Ind. Eng. Chem. (1986).
 */

namespace ctrl {

/**
 * @brief Identified SOPDT model parameters.
 */
struct SOPDTModel
{
    double K;        ///< Process gain [output_unit / input_unit].
    double tau1;     ///< Dominant time constant [s] (tau1 >= tau2).
    double tau2;     ///< Fast time constant [s].
    double theta;    ///< Dead time [s].
    double fitRMSE;  ///< RMS residual over the identification data [output_unit].
};

/**
 * @brief SOPDT identification method selector.
 */
enum class SOPDTMethod
{
    Graphical,    ///< Tangent + 63.2% + 28.3% crossing method (fast).
    Optimization, ///< Nested golden-section least-squares (slower, more accurate).
};

/**
 * @brief Second-Order Plus Dead-Time step-response identifier.
 */
class SOPDTIdentifier
{
public:
    /**
     * @brief Construct from open-loop step-response data.
     *
     * @param times         Time vector [s], monotonically increasing.
     * @param outputs       Measured output vector (same length as times).
     * @param stepMagnitude Magnitude of the step change in the manipulated variable.
     * @param stepTime      Time at which the step was applied [s] (default 0).
     *
     * @throws std::invalid_argument On dimension mismatch, constant output, or insufficient data.
     */
    SOPDTIdentifier(const std::vector<double> &times,
                    const std::vector<double> &outputs,
                    double stepMagnitude,
                    double stepTime = 0.0);

    /**
     * @brief Identify SOPDT parameters (K, tau1, tau2, theta).
     *
     * @param method Identification method (default: Graphical).
     * @return SOPDTModel with K, tau1, tau2, theta, and fitRMSE.
     */
    SOPDTModel identify(SOPDTMethod method = SOPDTMethod::Graphical) const;

    /**
     * @brief Evaluate the fitted SOPDT model step response at time @p t.
     *
     * @param m Identified SOPDT model.
     * @param t Evaluation time [s].
     * @return Predicted output at time t.
     */
    double evaluate(const SOPDTModel &m, double t) const;

    /**
     * @brief IMC-PID tuning from a SOPDT model (Rivera 1986 extended for SOPDT).
     *
     * Uses the equivalent FOPDT approximation tau_eq = tau1 + tau2:
     * @code
     *   Kp = tau_eq / (K * (lambdaC + theta / 2))
     *   Ti = tau_eq
     *   Td = tau1 * tau2 / (tau1 + tau2)   [zero for PI mode]
     * @endcode
     *
     * Rule of thumb: lambdaC = (1..2) * max(tau_eq, theta) for moderate speed.
     *
     * @param model    Identified SOPDT model.
     * @param lambdaC  Desired closed-loop time constant [s]. Larger = more conservative.
     * @param Ts       Controller sample time [s].
     * @param piOnly   If true, Td is set to zero (PI mode).
     * @return DiscretePIDParams ready for DiscretePID construction.
     */
    static PIDParams imcTuning(const SOPDTModel &model, double lambdaC,
                               double Ts, bool piOnly = false);

private:
    std::vector<double> t_;     ///< Times relative to stepTime.
    std::vector<double> y_;     ///< Outputs shifted so y[0] = 0.
    double              step_;  ///< stepMagnitude.
    double              y0_;    ///< Initial output level.
    double              Ts_data_; ///< Average sample interval [s].

    SOPDTModel identifyGraphical() const;
    SOPDTModel identifyOptimization() const;

    /// Evaluate SOPDT step response (zero-referenced, normalized by step_).
    /// Returns K * step_ * sopdt_unitstep(t, tau1, tau2) for t > theta, else 0.
    double sopdtResponse(double t, double K_norm, double tau1, double tau2,
                         double theta) const;

    /// SSE of SOPDT model over data.
    double modelSSE(double K_norm, double tau1, double tau2, double theta) const;

    /// 1-D golden-section minimum search on [a, b].
    static double goldenSection(std::function<double(double)> f,
                                double a, double b, double tol = 1e-5);
};

} // namespace ctrl
