#pragma once
#include "DiscretePID.h"
#include <functional>
#include <vector>
#include <stdexcept>

/**
 * @file FOPDTIdentifier.h
 * @brief First-Order Plus Dead-Time (FOPDT) step-response identification.
 *
 * Fits the continuous-time model:
 * @code
 *   G(s) = K * exp(-theta * s) / (tau * s + 1)
 * @endcode
 * to an open-loop step-response data set (time vector, measured output).
 *
 * **Two identification methods:**
 *
 * 1. **Graphical (Ziegler-Nichols tangent method, default):**
 *    - K    = (y_final - y_initial) / stepMagnitude
 *    - theta = intercept of the steepest tangent with the y=y_initial line
 *    - tau   = t_{63.2%} - theta  (time constant = time to 63.2% of final value)
 *    - Fast, robust, used in PDC Module 06 (pdc-master reference).
 *
 * 2. **Optimization (least-squares refinement):**
 *    - Golden-section search on (theta, tau) pairs minimising SSE over the data.
 *    - More accurate for noisy data or unusual response shapes.
 *
 * **IMC-PID tuning rule (Rivera 1986):**
 * Given the identified model and a desired closed-loop speed lambda_c [s]:
 * @code
 *   Kp  = (tau + theta/2) / (K * (lambda_c + theta/2))
 *   Ti  = tau + theta/2         [s]
 *   Td  = tau * theta / (2*tau + theta)  [s]   (PID; set to 0 for PI)
 * @endcode
 *
 * **Usage:**
 * @code
 *   ctrl::FOPDTIdentifier id(t_vec, y_vec, step_mag, step_time);
 *   auto m = id.identify();
 *   std::cout << "K=" << m.K << " tau=" << m.tau << " theta=" << m.theta << "\n";
 *
 *   // Auto-tune PID with IMC rule, lambda_c = 2*theta:
 *   auto pp = ctrl::FOPDTIdentifier::imcTuning(m, 2.0 * m.theta, Ts);
 *   ctrl::DiscretePID pid(pp, Ts);
 * @endcode
 *
 * @see pdc-master/Module_06/1_Topic/FirstOrderPlusDeadTime_1.py
 * @see Rivera, Morari & Skogestad, "Internal Model Control", Ind. Eng. Chem. (1986).
 * @see Ziegler & Nichols, "Optimum Settings for Automatic Controllers", ASME (1942).
 */

namespace ctrl {

/**
 * @brief Identified FOPDT model parameters.
 */
struct FOPDTModel
{
    double K;        ///< Process gain [output_unit / input_unit].
    double tau;      ///< Time constant [s]. Process variable reaches 63.2% of K at t=theta+tau.
    double theta;    ///< Dead time [s]. Output is flat for t in [t_step, t_step + theta].
    double fitRMSE;  ///< RMS residual of the model fit over the identification data [output_unit].
};

/**
 * @brief FOPDT identification method selector.
 */
enum class FOPDTMethod
{
    Graphical,    ///< Tangent + 63.2% intercept (fast; PDC Module 06 graphical method).
    Optimization, ///< Golden-section least-squares refinement (slower; more accurate).
};

/**
 * @brief First-Order Plus Dead-Time step-response identifier.
 */
class FOPDTIdentifier
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
     * @throws std::invalid_argument On dimension mismatch, constant output, or empty data.
     */
    FOPDTIdentifier(const std::vector<double> &times,
                    const std::vector<double> &outputs,
                    double stepMagnitude,
                    double stepTime = 0.0);

    /**
     * @brief Identify FOPDT parameters (K, tau, theta).
     *
     * @param method Identification method (default: Graphical).
     * @return FOPDTModel with K, tau, theta, and fitRMSE.
     */
    FOPDTModel identify(FOPDTMethod method = FOPDTMethod::Graphical) const;

    /**
     * @brief Evaluate the fitted FOPDT model step response at time @p t.
     *
     * Returns @c y0 + K * stepMagnitude * (1 - exp(-(t - stepTime - theta) / tau))
     * for t >= stepTime + theta, otherwise y0.
     *
     * @param m Identified FOPDT model.
     * @param t Evaluation time [s].
     * @return Predicted output at time t.
     */
    double evaluate(const FOPDTModel &m, double t) const;

    /**
     * @brief IMC-PID tuning from a FOPDT model (Rivera 1986).
     *
     * Yields a PID controller with:
     * @code
     *   Kp = (tau + theta/2) / (K * (lambdaC + theta/2))
     *   Ti = tau + theta/2
     *   Td = tau * theta / (2 * tau + theta)   [set to 0 for PI mode]
     * @endcode
     *
     * Rule of thumb: lambdaC = (1..3) * max(theta, tau) gives moderate speed;
     * lambdaC = theta/2 for aggressive response.
     *
     * @param model    Identified FOPDT model.
     * @param lambdaC  Closed-loop time constant [s]. Larger = more conservative.
     * @param Ts       Controller sample time [s].
     * @param piOnly   If true, Td is set to zero (PI instead of PID).
     * @return DiscretePIDParams ready for DiscretePID construction.
     */
    static PIDParams imcTuning(const FOPDTModel &model, double lambdaC,
                               double Ts, bool piOnly = false);

private:
    std::vector<double> t_;     // times relative to stepTime
    std::vector<double> y_;     // outputs (shifted so y[0] = 0)
    double              step_;  // stepMagnitude
    double              y0_;    // initial output level
    double              Ts_data_; // average sample interval [s]

    FOPDTModel identifyGraphical() const;
    FOPDTModel identifyOptimization() const;

    // SSE of FOPDT model (K_norm=1) given (theta, tau) over data
    double modelSSE(double K_norm, double tau, double theta) const;

    // 1-D golden-section search minimising f(x) over [a,b]
    static double goldenSection(std::function<double(double)> f,
                                double a, double b, double tol = 1e-5);
};

} // namespace ctrl
