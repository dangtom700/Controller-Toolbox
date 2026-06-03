#pragma once
#include "IController.h"
#include <Eigen/Dense>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

namespace ctrl {

/**
 * @brief Control Barrier Function (CBF) safety filter.
 *
 * Ames, Xu, Grizzle & Tabuada, *IEEE TAC* 2017.
 *
 * Wraps any existing `IController` and modifies its output minimally to ensure
 * the system state stays inside a user-defined safe set S = {x : h(x) >= 0}.
 *
 * The safety constraint is:
 * @code
 *   dh/dx . f(x, u) >= -alpha . h(x)    (CBF condition)
 * @endcode
 * which, for an affine-in-control system xdot = f_0(x) + g(x).u, becomes the
 * linear constraint:
 * @code
 *   (dh/dx . g(x)) . u >= -dh/dx . f_0(x) - alpha . h(x)
 * @endcode
 *
 * The safety filter solves a **1D QP** each step:
 * @code
 *   u_safe = argmin_u  (1/2)(u - u_nom)^2
 *            s.t.      A_cbf . u >= b_cbf
 *                      u_min <= u <= u_max
 * @endcode
 * which has an analytical solution: `u_safe = clamp(u_nom, u_min_cbf, u_max)`.
 *
 * **Usage (SISO):**
 * @code
 *   // Safe set: h(x) = x_max - x  (upper bound on state)
 *   auto h_fn    = [x_max](double x) { return x_max - x; };
 *   auto grad_fn = [](double)        { return -1.0; };   // dh/dx
 *   auto f0_fn   = [a](double x)     { return a * x; };  // drift
 *   auto g_fn    = []( double)       { return 1.0; };    // input gain
 *
 *   ctrl::CBFSafetyFilter::Params p;
 *   p.alpha = 1.0;  p.uMin = -2.0;  p.uMax = 2.0;
 *   ctrl::CBFSafetyFilter cbf(pid, h_fn, grad_fn, f0_fn, g_fn, p, Ts);
 *
 *   cbf.setState(x);
 *   double u_safe = cbf.compute(r - x);
 * @endcode
 *
 * @note This filter is SISO (single-input single-state).
 *       For MIMO systems, define h(x) as a scalar function of the state vector.
 *
 * @see Ames, A.D. et al. (2017). Control barrier function based quadratic programs
 *      for safety critical systems. IEEE TAC 62(8), 3861-3876.
 */
class CBFSafetyFilter : public IController {
public:
    using ScalarFn  = std::function<double(double)>;  ///< Scalar function of scalar state

    struct Params {
        double alpha = 1.0;     ///< CBF decay rate alpha (larger = more conservative)
        double uMin  = -1e9;    ///< Hard lower bound on output
        double uMax  =  1e9;    ///< Hard upper bound on output
        double slack = 0.0;     ///< Relaxation epsilon: replaces h(x)>=0 with h(x)+epsilon>=0 (soft CBF)
    };

    /**
     * @brief Construct a CBF safety filter wrapping a nominal controller.
     *
     * @param nominal  The nominal controller whose output will be modified if needed.
     * @param h        Barrier function h(x) >= 0 defines the safe set.
     * @param dh_dx    Gradient dh/dx of the barrier function.
     * @param f0       Drift term f0(x) of the control-affine dynamics xdot=f0+g.u.
     * @param g        Input gain g(x) of the control-affine dynamics.
     * @param params   CBF parameters.
     * @param Ts       Sample time [s].
     */
    CBFSafetyFilter(std::shared_ptr<IController> nominal,
                    ScalarFn h,
                    ScalarFn dh_dx,
                    ScalarFn f0,
                    ScalarFn g,
                    const Params& params,
                    double Ts);

    /** @brief Set the current plant state for the CBF condition evaluation. */
    void setState(double x) noexcept { x_ = x; }

    // ---- IController -------------------------------------------------------
    /**
     * @brief Compute safety-filtered control action.
     * @param error Tracking error (passed through to the nominal controller).
     * @return Safety-filtered u.
     */
    double compute(double error) override;

    void        reset()              override;
    double      sampleTime()   const override { return Ts_; }
    std::string name()         const override { return "CBFSafetyFilter"; }

    // ---- Diagnostics -------------------------------------------------------
    /** @brief true if the CBF constraint was active on the last call. */
    bool   cbfActive()     const noexcept { return cbf_active_; }
    /** @brief Current barrier function value h(x). */
    double barrierValue()  const noexcept { return h_val_; }
    /** @brief Nominal controller output before safety modification. */
    double nominalOutput() const noexcept { return u_nom_; }

private:
    std::shared_ptr<IController> nominal_;
    ScalarFn h_, dh_dx_, f0_, g_;
    Params   p_;
    double   Ts_;

    double x_        = 0.0;
    bool   cbf_active_ = false;
    double h_val_    = 0.0;
    double u_nom_    = 0.0;
};

} // namespace ctrl
