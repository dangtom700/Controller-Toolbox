#pragma once
#include "ControllerRegistry.h"
#include <Eigen/Dense>
#include <functional>

namespace ctrl {

/**
 * @brief Grey-box nonlinear parameter estimator via Levenberg-Marquardt (E1).
 *
 * Identifies unknown physical parameters p in a continuous-time ODE:
 *   x_dot = f(x, u, p),   y = h(x, p)
 *
 * Given initial state x0, input sequence U (n_u x N), and measurements Y (n_y x N),
 * minimises  sum_{k=0}^{N-1} ||y_k - h(x_k(p), p)||^2  w.r.t. p via
 * Levenberg-Marquardt with optional per-parameter bounds.
 *
 * Integration uses RK4 (rk4_steps substeps per Ts).
 * Parameter Jacobian is computed by central finite differences.
 * Bounds are enforced by projection after each LM step.
 *
 * @par Typical usage
 * @code
 *   // Spring-mass-damper: xdot = [x1, -k/m*x0 - c/m*x1 + u/m]
 *   ctrl::GreyBoxEstimator::OdeFn f = [](auto& x, auto& u, auto& p) {
 *       Eigen::VectorXd xd(2);
 *       xd(0) = x(1);
 *       xd(1) = -p(0)/p(2)*x(0) - p(1)/p(2)*x(1) + u(0)/p(2);
 *       return xd;
 *   };
 *   ctrl::GreyBoxEstimator::MeasFn h = [](auto& x, auto& p) {
 *       return x.head(1);  // observe position
 *   };
 *   ctrl::GreyBoxEstimator::Params par;
 *   par.p0 = Eigen::Vector3d(5.0, 0.5, 1.0);  // k, c, m initial guess
 *   par.lower = Eigen::Vector3d(0.1, 0.01, 0.1);
 *   par.upper = Eigen::Vector3d(20.0, 5.0, 5.0);
 *   par.Ts = 0.01;
 *   ctrl::GreyBoxEstimator est(f, h, par);
 *   auto res = est.fit(x0, U, Y);
 * @endcode
 */
class GreyBoxEstimator {
public:
    /** @brief ODE: xdot = f(x, u, p). */
    using OdeFn  = std::function<Eigen::VectorXd(
        const Eigen::VectorXd& x,
        const Eigen::VectorXd& u,
        const Eigen::VectorXd& p)>;

    /** @brief Measurement: y = h(x, p). */
    using MeasFn = std::function<Eigen::VectorXd(
        const Eigen::VectorXd& x,
        const Eigen::VectorXd& p)>;

    struct Params {
        Eigen::VectorXd p0;              ///< Initial parameter guess (n_p x 1)
        Eigen::VectorXd lower;           ///< Per-element lower bound (empty = unconstrained)
        Eigen::VectorXd upper;           ///< Per-element upper bound (empty = unconstrained)
        double Ts         = 0.01;        ///< Sample / integration time [s]
        int    rk4_steps  = 1;           ///< RK4 substeps per Ts (increase for stiff ODE)
        double lm_lambda0 = 1e-3;        ///< Initial LM damping factor
        double lm_nu      = 10.0;        ///< LM damping scale factor (> 1)
        int    max_iter   = 100;         ///< Maximum LM iterations
        double tol_grad   = 1e-8;        ///< Converge when max|J'r| < tol_grad
        double eps_jac    = 1e-5;        ///< FD step scale for parameter Jacobian
    };

    struct Result {
        Eigen::VectorXd params;      ///< Best-fit parameter vector (n_p x 1)
        double          cost;        ///< Final cost 0.5 * ||r||^2
        int             iterations;  ///< Number of LM iterations used
        bool            converged;   ///< True if gradient convergence criterion met
    };

    /**
     * @brief Construct estimator.
     * @param ode  Continuous-time ODE f(x, u, p) -> xdot.
     * @param meas Measurement function h(x, p) -> y.
     * @param par  Parameters including initial guess p0.
     */
    GreyBoxEstimator(OdeFn ode, MeasFn meas, const Params& par);

    /**
     * @brief Run Levenberg-Marquardt parameter estimation.
     * @param x0  Initial state (n_x x 1).
     * @param U   Input sequence (n_u x N); column k = u[k].
     * @param Y   Measurement sequence (n_y x N); column k = y_meas[k].
     * @return    Estimated parameters and convergence diagnostics.
     */
    Result fit(const Eigen::VectorXd& x0,
               const Eigen::MatrixXd& U,
               const Eigen::MatrixXd& Y);

    /**
     * @brief Simulate and return predicted outputs using the current estimated params.
     * @param x0  Initial state (n_x x 1).
     * @param U   Input sequence (n_u x N).
     * @return    Predicted output trajectory Y_hat (n_y x N).
     */
    Eigen::MatrixXd predict(const Eigen::VectorXd& x0,
                             const Eigen::MatrixXd& U) const;

    /** @brief Current (last fitted) parameter estimate. */
    const Eigen::VectorXd& params()   const { return p_est_; }

    /** @brief True after a successful fit() call. */
    bool                   isFitted() const { return fitted_; }

private:
    Eigen::VectorXd rk4Step(const Eigen::VectorXd& x,
                             const Eigen::VectorXd& u,
                             const Eigen::VectorXd& p) const;

    Eigen::MatrixXd simulateY(const Eigen::VectorXd& x0,
                               const Eigen::MatrixXd& U,
                               const Eigen::VectorXd& p) const;

    Eigen::VectorXd computeResidual(const Eigen::VectorXd& x0,
                                     const Eigen::MatrixXd& U,
                                     const Eigen::MatrixXd& Y,
                                     const Eigen::VectorXd& p) const;

    Eigen::MatrixXd computeJacobianFD(const Eigen::VectorXd& x0,
                                       const Eigen::MatrixXd& U,
                                       const Eigen::MatrixXd& Y,
                                       const Eigen::VectorXd& p) const;

    Eigen::VectorXd clipParams(const Eigen::VectorXd& p) const;

    OdeFn           f_;
    MeasFn          h_;
    Params          par_;
    Eigen::VectorXd p_est_;
    bool            fitted_ = false;
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(grey_box_estimator)
