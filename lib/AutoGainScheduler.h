#pragma once
#include "GainScheduledController.h"
#include "GapMetric.h"
#include "LinearModelCluster.h"
#include "LinearisationHelper.h"
#include <functional>
#include <memory>
#include <vector>
#include <stdexcept>
#include <cmath>
#include <iostream>
#include <Eigen/Dense>

/**
 * @file AutoGainScheduler.h
 * @brief Automated gain-scheduled controller builder via gap-metric clustering.
 *
 * Implements the full pipeline:
 *
 * 1. **Grid sweep**: evaluate the nonlinear plant dynamics at `grid_density`
 *    uniformly-spaced scheduling-parameter values over [p_min, p_max].
 * 2. **Equilibrium finder**: for each p, solve f(x, u_eq(p)) = 0 for x using
 *    Newton-Raphson with the existing jacobianX() numerical Jacobian.
 * 3. **Linearisation**: call lineariseAtPoint() at each equilibrium.
 * 4. **Gap-metric clustering**: compute the nu-gap matrix and cluster models
 *    with single-linkage agglomerative clustering at the given threshold.
 * 5. **Controller design**: call the user-supplied design function for each
 *    cluster representative.
 * 6. **Assembly**: build and return a GainScheduledController with one
 *    controller per cluster, placed at the representative's scheduling value.
 *
 * @par Example
 * @code
 *   // Pendulum: thetadot = omega, omegadot = -(g/L)*sin(theta) + u/mL^2
 *   ctrl::StateFunc pend = [](const Eigen::VectorXd& x, const Eigen::VectorXd& u){
 *       Eigen::VectorXd xd(2);
 *       xd(0) = x(1);
 *       xd(1) = -9.81 * std::sin(x(0)) + u(0);
 *       return xd;
 *   };
 *
 *   // u_eq = m*g*L*sin(theta), x_eq = (theta, 0)
 *   auto u_eq_fn = [](double p) {
 *       Eigen::VectorXd u(1); u(0) = 9.81 * std::sin(p); return u; };
 *   auto x0_fn   = [](double p) {
 *       Eigen::VectorXd x(2); x(0) = p; x(1) = 0.0; return x; };
 *
 *   Eigen::VectorXd x_current(2);  // updated by the simulation loop
 *   auto design_fn = [&x_current](const ctrl::StateSpace& sys, double) -> std::shared_ptr<ctrl::IController> {
 *       ctrl::LQRParams lp;
 *       lp.Q = 10 * Eigen::MatrixXd::Identity(2, 2);
 *       lp.R = Eigen::MatrixXd::Identity(1, 1);
 *       // makeLQRController() owns the DiscreteLQR so no dangling reference arises.
 *       return ctrl::makeLQRController(sys, lp, [&x_current]{ return x_current; });
 *   };
 *
 *   auto sched = ctrl::buildAutoGainScheduler(
 *       pend, -1.0, 1.0, 10, u_eq_fn, x0_fn, design_fn, Ts);
 *
 *   sched->setSchedulingParam(theta_measured);
 *   double u = sched->compute(theta_ref - theta_measured);
 * @endcode
 *
 * @note The design_fn must return an IController subclass wrapped in a
 *   std::shared_ptr. All IController subclasses already satisfy this.
 */

namespace ctrl {

/**
 * @brief Equilibrium point of a continuous-time nonlinear system.
 */
struct OperatingPoint {
    Eigen::VectorXd x_eq; ///< Equilibrium state: f(x_eq, u_eq) = 0.
    Eigen::VectorXd u_eq; ///< Equilibrium input (provided by the user).
    double          p;    ///< Scheduling parameter value.
};

/**
 * @brief Solve f(x, u_eq) = 0 for x using Newton-Raphson.
 *
 * Iterates  x <- x - J_f^{-1}(x, u_eq) * f(x, u_eq)  using the numerical
 * Jacobian from jacobianX(). Converges in O(n_iter) steps for smooth f.
 *
 * @param f        Continuous-time dynamics xdot = f(x, u).
 * @param u_eq     Fixed equilibrium input.
 * @param x0       Initial guess for x_eq.
 * @param max_iter Newton iteration limit (default 50).
 * @param tol      Convergence: ||f(x, u)|| < tol (default 1e-8).
 * @return         Equilibrium state x_eq.
 * @throws std::runtime_error If Newton-Raphson fails to converge.
 */
inline Eigen::VectorXd findEquilibrium(const StateFunc&        f,
                                        const Eigen::VectorXd& u_eq,
                                        const Eigen::VectorXd& x0,
                                        int    max_iter = 50,
                                        double tol      = 1e-8)
{
    Eigen::VectorXd x = x0;
    for (int iter = 0; iter < max_iter; ++iter) {
        Eigen::VectorXd fx = f(x, u_eq);
        if (fx.norm() < tol) return x;
        Eigen::MatrixXd J = jacobianX(f, x, u_eq);
        // Solve J*dx = -f(x) via QR; fallback to pseudoinverse for degenerate J.
        Eigen::VectorXd dx = J.colPivHouseholderQr().solve(-fx);
        x += dx;
    }
    // Accept if residual is within a loose tolerance
    if (f(x, u_eq).norm() < tol * 1e3)
    {
#ifndef NDEBUG
        std::clog << "[findEquilibrium] WARNING: Newton-Raphson converged only within the "
                     "loose tolerance (tol * 1e3 = " << tol * 1e3 << "). "
                     "The returned equilibrium may be inaccurate. "
                     "Consider providing a better initial guess x0 or increasing max_iter.\n";
#endif
        return x;
    }
    throw std::runtime_error(
        "findEquilibrium: Newton-Raphson failed to converge. "
        "Check that f(x_eq, u_eq)=0 has a solution near x0.");
}

/**
 * @brief Build a gain-scheduled controller automatically from a nonlinear plant model.
 *
 * @param f              Continuous-time dynamics xdot = f(x, u).
 * @param p_min          Lower bound of scheduling parameter.
 * @param p_max          Upper bound of scheduling parameter.
 * @param grid_density   Number of grid points to sweep (>= 2).
 * @param u_eq_fn        p -> equilibrium input u_eq(p).
 * @param x0_fn          p -> initial guess for x_eq.
 * @param design_fn      (sys, p) -> shared_ptr<IController> for each cluster rep.
 * @param Ts             Discrete-time sample period [s].
 * @param gap_threshold  Maximum nu-gap within a cluster (default 0.5).
 * @param freq_points    Gap-metric frequency resolution (default 200).
 * @return               Assembled GainScheduledController (LinearBlend mode).
 * @throws std::invalid_argument If grid_density < 2.
 * @throws std::runtime_error   If any equilibrium search fails to converge.
 */
inline std::shared_ptr<GainScheduledController>
buildAutoGainScheduler(
    const StateFunc& f,
    double p_min, double p_max, int grid_density,
    const std::function<Eigen::VectorXd(double p)>& u_eq_fn,
    const std::function<Eigen::VectorXd(double p)>& x0_fn,
    const std::function<std::shared_ptr<IController>(const StateSpace&, double p)>& design_fn,
    double Ts,
    double gap_threshold = 0.5,
    int    freq_points   = 200)
{
    if (grid_density < 2)
        throw std::invalid_argument("buildAutoGainScheduler: grid_density must be >= 2.");

    // ---- Phase 1-2: grid sweep + equilibrium + linearisation ----------------
    std::vector<double>       p_grid(grid_density);
    std::vector<StateSpace>   models;
    models.reserve(grid_density);

    for (int i = 0; i < grid_density; ++i) {
        double p    = p_min + (p_max - p_min) * i / (grid_density - 1);
        p_grid[i]   = p;

        Eigen::VectorXd u_eq = u_eq_fn(p);
        Eigen::VectorXd x0   = x0_fn(p);
        Eigen::VectorXd x_eq = findEquilibrium(f, u_eq, x0);

        models.push_back(lineariseAtPoint(f, x_eq, u_eq, Ts));
    }

    // ---- Phase 3-4: nu-gap matrix + clustering --------------------------------
    Eigen::MatrixXd  G        = nuGapMatrix(models, freq_points);
    ClusterResult    clusters  = clusterByGap(G, gap_threshold);

    // ---- Phase 5-6: design + assemble ----------------------------------------
    auto scheduler = std::make_shared<GainScheduledController>(
                         Ts, GainScheduleMode::LinearBlend);

    for (int c = 0; c < clusters.numClusters; ++c) {
        int    rep_idx = clusters.representatives[c];
        double p_rep   = p_grid[rep_idx];
        auto   ctrl    = design_fn(models[rep_idx], p_rep);
        scheduler->addSchedulePoint(p_rep, ctrl);
    }

    return scheduler;
}

} // namespace ctrl
