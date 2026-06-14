#pragma once
#include "AutoTuner.h"      // TunerResult, CostFn
#include "GaussianProcess.h"
#include "ControllerRegistry.h"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

/**
 * @file BayesianOptimizer.h
 * @brief Header-only Bayesian Optimization (BO) for controller parameter tuning.
 *
 * Uses a Gaussian Process (GP) surrogate model of the cost function to select
 * the most informative next evaluation point, trading off exploration (high
 * uncertainty) and exploitation (low predicted cost) via an acquisition function.
 *
 * **Why BO over CMA-ES (AutoTuner)?**
 * CMA-ES evaluates `lambda ~ 4+3*ln(n)` points per generation - efficient for
 * cheap cost functions but wasteful when each evaluation requires a full
 * closed-loop simulation or hardware test. BO minimizes the number of cost
 * function evaluations (typically 20-100 total) while still finding good optima.
 * It is the algorithm of choice when evaluations are expensive or noisy.
 *
 * **Algorithm:**
 * @code
 *   1. Evaluate n_init random points (LHS-like random grid)
 *   2. Fit GP on {(x_i, cost_i)} observations
 *   3. For each BO iteration:
 *       a. argmin_x acquisition(x)   via random grid + local refinement
 *       b. Evaluate cost(x_new)
 *       c. GP.addPoint(x_new, cost_new); GP.fit()
 * @endcode
 *
 * **Acquisition functions:**
 * - **UCB** (default): `mu(x) - kappa*sigma(x)` - lower confidence bound.
 *   `kappa > 0` controls explore/exploit tradeoff. Larger kappa = more exploration.
 * - **EI**: `E[max(f_best - f(x), 0)]` - expected improvement over current best.
 *   More data-efficient when the noise level is well-estimated.
 *
 * **Usage:**
 * @code
 *   ctrl::BayesOptParams bp;
 *   bp.n = 3;                          // Kp, Ki, Kd
 *   bp.lower = Eigen::Vector3d(0.0, 0.0, 0.0);
 *   bp.upper = Eigen::Vector3d(10.0, 2.0, 1.0);
 *   bp.n_init    = 10;    // random exploration
 *   bp.maxIter   = 40;    // BO iterations  -> 50 cost fn calls total
 *   bp.kappa     = 2.0;   // UCB exploration weight
 *
 *   ctrl::BayesianOptimizer bo(bp);
 *   ctrl::TunerResult r = bo.tune(
 *       [&](const Eigen::VectorXd& p) {
 *           // run closed-loop simulation and return IAE
 *           return simulate_iae(p);
 *       }, x0);
 *
 *   std::cout << "Best IAE = " << r.cost << "\n";
 * @endcode
 *
 * @note Shares `TunerResult` and `CostFn` types with AutoTuner so the same
 *       objective function and result struct work with both optimizers.
 *
 * @see AutoTuner  -- CMA-ES for cheap cost functions (1000+ evals acceptable).
 * @see Rasmussen & Williams (2006). Gaussian Processes for Machine Learning.
 * @see Srinivas et al. (2010). Gaussian Process Optimization in the Bandit Setting:
 *      No Regret and Experimental Design. ICML.
 */

namespace ctrl {

/**
 * @brief Parameters for BayesianOptimizer.
 */
struct BayesOptParams
{
    int  n              = 1;     ///< Parameter space dimension.

    int  n_init         = 10;    ///< Initial random evaluations before GP is fitted.
    int  maxIter        = 50;    ///< BO iterations after initialisation.
    int  n_acq_restarts = 200;   ///< Candidate points for acquisition maximisation.

    double kappa        = 2.0;   ///< UCB exploration weight (larger = more exploration).
    double xi           = 0.01;  ///< EI improvement target margin.

    /** @brief Acquisition function selector. */
    enum class Acquisition { UCB, EI } acq = Acquisition::UCB;

    /**
     * @brief Lower bounds (required). Must be strictly less than upper.
     */
    Eigen::VectorXd lower;
    /**
     * @brief Upper bounds (required). Must be strictly greater than lower.
     */
    Eigen::VectorXd upper;

    /**
     * @brief GP surrogate hyperparameters.
     *
     * The optimizer normalises inputs to [0,1]^n before passing to the GP.
     * A length scale of ~0.3 in normalised space is a reasonable default for
     * parameter spaces with typical smoothness.
     */
    GaussianProcess::Params gp;

    unsigned int seed = 42;     ///< RNG seed for reproducibility.

    BayesOptParams() {
        gp.length_scale = 0.3;
        gp.signal_var   = 1.0;
        gp.noise_var    = 1e-4;
        gp.n_max        = 0;    // unlimited - BO typically has < 200 evaluations
    }
};

/**
 * @brief Bayesian Optimization with GP surrogate and UCB/EI acquisition.
 *
 * Uses `TunerResult` and `CostFn` from AutoTuner.h for API compatibility -
 * the same objective function works with both AutoTuner and BayesianOptimizer.
 */
class BayesianOptimizer
{
public:
    /** @brief Cost function type (same as AutoTuner::CostFn). */
    using CostFn = AutoTuner::CostFn;

    /**
     * @brief Construct optimizer.
     * @param p    BO parameters including dimension, bounds, and GP config.
     * @throws std::invalid_argument if n < 1 or bounds are inconsistent.
     */
    explicit BayesianOptimizer(const BayesOptParams& p)
        : p_(p), rng_(p.seed)
    {
        if (p_.n < 1)
            throw std::invalid_argument("BayesianOptimizer: n must be >= 1.");
        if (p_.lower.size() != p_.n || p_.upper.size() != p_.n)
            throw std::invalid_argument(
                "BayesianOptimizer: lower/upper must have n elements.");
        if ((p_.lower.array() >= p_.upper.array()).any())
            throw std::invalid_argument(
                "BayesianOptimizer: lower must be < upper element-wise.");
    }

    /**
     * @brief Run Bayesian Optimization.
     *
     * Performs `n_init` random evaluations then `maxIter` BO iterations.
     * Total cost function evaluations = n_init + maxIter.
     *
     * @param cost User cost function f(params) -> non-negative double.
     * @param x0   Starting point (used as first evaluation; must satisfy bounds).
     * @return     TunerResult with best params and cost found.
     */
    TunerResult tune(CostFn cost, const Eigen::VectorXd& x0);

private:
    // Normalise x in [lower, upper] to [0,1]^n
    Eigen::VectorXd normalise(const Eigen::VectorXd& x) const;
    // Denormalise from [0,1]^n to [lower, upper]
    Eigen::VectorXd denormalise(const Eigen::VectorXd& xn) const;
    // Clamp to [lower, upper]
    Eigen::VectorXd clamp(const Eigen::VectorXd& x) const;

    // Evaluate UCB acquisition (negative = better for argmin)
    double acqUCB(const GaussianProcess& gp, const Eigen::VectorXd& xn) const;
    // Evaluate EI acquisition (negative = better for argmin)
    double acqEI(const GaussianProcess& gp, const Eigen::VectorXd& xn,
                  double f_best) const;

    // Sample n_acq_restarts random points in [0,1]^n, return the argmin of acquisition
    Eigen::VectorXd maximiseAcquisition(const GaussianProcess& gp,
                                         double f_best);

    BayesOptParams   p_;
    std::mt19937     rng_;
};

// =============================================================================
// Implementation (header-only)
// =============================================================================

inline Eigen::VectorXd BayesianOptimizer::normalise(const Eigen::VectorXd& x) const
{
    return (x - p_.lower).cwiseQuotient(p_.upper - p_.lower);
}

inline Eigen::VectorXd BayesianOptimizer::denormalise(const Eigen::VectorXd& xn) const
{
    return p_.lower + xn.cwiseProduct(p_.upper - p_.lower);
}

inline Eigen::VectorXd BayesianOptimizer::clamp(const Eigen::VectorXd& x) const
{
    return x.cwiseMax(p_.lower).cwiseMin(p_.upper);
}

inline double BayesianOptimizer::acqUCB(const GaussianProcess& gp,
                                          const Eigen::VectorXd& xn) const
{
    auto pred = gp.predict(xn);
    return pred.mean - p_.kappa * std::sqrt(std::max(pred.variance, 0.0));
}

inline double BayesianOptimizer::acqEI(const GaussianProcess& gp,
                                         const Eigen::VectorXd& xn,
                                         double f_best) const
{
    auto pred = gp.predict(xn);
    const double sigma = std::sqrt(std::max(pred.variance, 1e-12));
    const double z     = (f_best - pred.mean - p_.xi) / sigma;
    static constexpr double kInvSqrt2Pi = 0.3989422804014327;
    const double phi   = kInvSqrt2Pi * std::exp(-0.5 * z * z);
    const double Phi   = 0.5 * (1.0 + std::erf(z * 0.7071067811865476));
    // EI = sigma * (phi + z*Phi); negate to turn argmax into argmin
    return -(sigma * (phi + z * Phi));
}

inline Eigen::VectorXd
BayesianOptimizer::maximiseAcquisition(const GaussianProcess& gp,
                                        double f_best)
{
    std::uniform_real_distribution<double> uniform(0.0, 1.0);

    double best_acq = std::numeric_limits<double>::infinity();
    Eigen::VectorXd best_xn = Eigen::VectorXd::Constant(p_.n, 0.5);

    Eigen::VectorXd xn(p_.n);
    for (int i = 0; i < p_.n_acq_restarts; ++i) {
        for (int j = 0; j < p_.n; ++j)
            xn(j) = uniform(rng_);

        double acq_val = (p_.acq == BayesOptParams::Acquisition::UCB)
                         ? acqUCB(gp, xn)
                         : acqEI(gp, xn, f_best);

        if (acq_val < best_acq) {
            best_acq = acq_val;
            best_xn  = xn;
        }
    }
    return best_xn;
}

inline TunerResult BayesianOptimizer::tune(CostFn cost, const Eigen::VectorXd& x0)
{
    const int n = p_.n;
    GaussianProcess gp(n, p_.gp);

    double best_cost = std::numeric_limits<double>::infinity();
    Eigen::VectorXd best_params = clamp(x0);
    int n_evals = 0;

    std::uniform_real_distribution<double> uniform(0.0, 1.0);

    // -----------------------------------------------------------------------
    // Phase 1: random initialisation  (includes x0 as the first point)
    // -----------------------------------------------------------------------
    auto evaluate = [&](const Eigen::VectorXd& xn) {
        Eigen::VectorXd x = clamp(denormalise(xn));
        double c = cost(x);
        ++n_evals;
        gp.addPoint(xn, c);
        if (c < best_cost) {
            best_cost   = c;
            best_params = x;
        }
    };

    // Evaluate x0 first
    evaluate(normalise(best_params));

    // Fill remaining n_init slots with random points
    Eigen::VectorXd xn(n);
    for (int i = 1; i < p_.n_init; ++i) {
        for (int j = 0; j < n; ++j) xn(j) = uniform(rng_);
        evaluate(xn);
    }

    // -----------------------------------------------------------------------
    // Phase 2: GP-guided BO iterations
    // -----------------------------------------------------------------------
    for (int iter = 0; iter < p_.maxIter; ++iter) {
        gp.fit();

        // Acquisition maximisation (returns normalised x)
        Eigen::VectorXd xn_next = maximiseAcquisition(gp, best_cost);
        evaluate(xn_next);
    }

    return {best_params, best_cost, n_evals, p_.maxIter, true};
}

} // namespace ctrl

CTRL_REGISTER_FEATURE(bayesian_optimizer)
