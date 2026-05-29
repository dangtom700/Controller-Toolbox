#pragma once
#include <Eigen/Dense>
#include <functional>
#include <memory>
#include "IController.h"

/**
 * @file FeedbackLinearisation.h
 * @brief Discrete-time feedback linearisation controller (SISO, relative degree 1).
 *
 * Implements exact feedback linearisation for a continuous-time nonlinear system of the
 * affine-in-control form:
 * @code
 *   xdot = f_drift(x) + g_gain(x) . u       (continuous-time, relative degree 1)
 *   y = C.x   (or any scalar output)
 * @endcode
 *
 * The control law:
 * @code
 *   u[k] = clamp( (v[k] - f(x[k], u[k-1])) / g(x[k], u[k-1]),  uMin, uMax )
 * @endcode
 *
 * where v[k] is the "virtual control" computed by an inner linear IController (e.g., PID)
 * on the tracking error `e = r - y`.  After the nonlinear terms are cancelled the closed-loop
 * dynamics seen by the inner controller are approximately those of an integrator chain, which
 * can be stabilised with a simple pole-placement or PID inner loop.
 *
 * @par Feasibility conditions
 *  - **Relative degree 1**: the control input u appears in the first time derivative of y.
 *    If relative degree > 1, cascade the FL with an integrator or use backstepping.
 *  - **Minimum phase**: the zero dynamics (dynamics when y=0) must be asymptotically stable.
 *    Unstable zeros cause internal states to grow even when y tracks the reference.
 *  - **g(x) != 0** across the operating region. The `regularisationEps` parameter prevents
 *    division by zero when g is near zero; choose it smaller than the minimum expected |g|.
 *  - Full state x[k] must be available each step (measured or estimated via EKF/UKF/MHE).
 *    Call `setState(x)` before each `compute()`.
 *
 * @par Usage
 * @code
 *   // System: xdot = -x^3 + u  (f = -x^3, g = 1)
 *   auto f = [](const Eigen::VectorXd& x, double) { return -x(0)*x(0)*x(0); };
 *   auto g = [](const Eigen::VectorXd&, double)   { return 1.0; };
 *
 *   ctrl::PIDParams pp; pp.Kp = 5.0; pp.Ki = 2.0;
 *   auto inner = std::make_shared<ctrl::DiscretePID>(pp, Ts);
 *
 *   ctrl::FLParams flp; flp.uMin = -20.0; flp.uMax = 20.0;
 *   ctrl::FeedbackLinearisationController fl(f, g, inner, flp, Ts);
 *
 *   // Simulation loop
 *   Eigen::VectorXd x(1); x(0) = 0.0;
 *   for (int k = 0; k < N; ++k) {
 *       fl.setState(x);                       // inject current state
 *       double u = fl.compute(ref - x(0));    // error = r - y
 *       x(0) += Ts * (-x(0)*x(0)*x(0) + u);  // Euler integration (plant)
 *   }
 * @endcode
 *
 * @warning Perfect cancellation relies on an exact model for f and g.  Model error enters
 *   as an unmatched disturbance in the inner-loop error dynamics.  For robustly uncertain
 *   plants, use SMC (DiscreteSMC) or ADRC (DiscreteADRC) instead.
 */

namespace ctrl
{

/** @brief Tuning parameters for FeedbackLinearisationController. */
struct FLParams
{
    double uMin              = -1.0e6; ///< Lower saturation limit on the physical control u [engineering units].
    double uMax              =  1.0e6; ///< Upper saturation limit on the physical control u.
    double regularisationEps =  1.0e-6; ///< Minimum |g| before clamping: u = (v-f) / max(|g|, eps).
};

/**
 * @brief Feedback linearisation controller - SISO, relative degree 1.
 *
 * Implements `IController` so it can be used inside `ControllerStack` or passed to any
 * tuner or metric that accepts `IController` references.
 */
class FeedbackLinearisationController : public IController
{
public:
    /** @brief Drift term f in xdot = f(x) + g(x).u, evaluated at (state, previous_u). Scalar return. */
    using DriftFn = std::function<double(const Eigen::VectorXd &x, double u_prev)>;
    /** @brief Input gain g in xdot = f(x) + g(x).u, evaluated at (state, previous_u). Scalar return. */
    using GainFn  = std::function<double(const Eigen::VectorXd &x, double u_prev)>;

    /**
     * @brief Construct the feedback linearisation controller.
     * @param f       Drift functor: f(x, u_prev) -> double.
     * @param g       Gain functor:  g(x, u_prev) -> double (must be non-zero in operating region).
     * @param inner   Inner linear IController that computes virtual control v from tracking error.
     * @param params  Output saturation limits and regularisation epsilon.
     * @param Ts      Sample time [s].
     */
    FeedbackLinearisationController(DriftFn f,
                                    GainFn  g,
                                    std::shared_ptr<IController> inner,
                                    const FLParams &params,
                                    double Ts);

    /**
     * @brief Compute one control step.
     *
     * Execution order:
     *  1. v = inner_->compute(error)          [virtual control from inner loop]
     *  2. f_val = f_(x_, u_last_)             [drift at current state]
     *  3. g_val = g_(x_, u_last_)             [gain at current state, regularised]
     *  4. u = clamp((v - f_val) / g_eff, uMin, uMax)
     *  5. Store u_last_ = u; return u
     *
     * @param error  Tracking error e = r - y (or y - r for systems where g < 0, see note).
     * @return       Physical control signal u[k].
     *
     * @note Call setState(x) before each compute() with the current plant state.
     */
    double compute(double error) override;

    /**
     * @brief Reset inner controller and clear u_last_.
     */
    void reset() override;

    /** @brief Sample time Ts [s]. */
    double sampleTime() const override { return Ts_; }

    /**
     * @brief Inject the current plant state x[k].
     * Must be called once per sample before compute().
     * @param x State vector (n * 1). Size must match n at construction.
     */
    void setState(const Eigen::VectorXd &x) { x_ = x; }

    /** @brief Current state held by the controller (as last set by setState()). */
    const Eigen::VectorXd &state() const { return x_; }

    /** @brief Last physical control output u[k-1]. */
    double lastOutput() const { return u_last_; }

    /** @brief Update parameters at runtime (saturation / regularisation only). */
    void setParams(const FLParams &p) { params_ = p; }

    /** @brief Current parameter set. */
    const FLParams &params() const { return params_; }

    /** @brief Access the inner controller for re-tuning. */
    std::shared_ptr<IController> innerController() const { return inner_; }

private:
    DriftFn                      f_;
    GainFn                       g_;
    std::shared_ptr<IController> inner_;
    FLParams                     params_;
    double                       Ts_;
    Eigen::VectorXd              x_;     ///< Current state (set by setState).
    double                       u_last_ = 0.0;
};

} // namespace ctrl
