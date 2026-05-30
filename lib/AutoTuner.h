#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>
#include <random>
#include <vector>

/**
 * @file AutoTuner.h
 * @brief Header-only CMA-ES black-box optimizer for controller parameter tuning.
 *
 * Implements the Covariance Matrix Adaptation Evolution Strategy (CMA-ES),
 * a population-based derivative-free optimizer well suited to low-dimensional
 * control parameter spaces (Kp/Ki/Kd, MPC weights, horizon lengths).
 *
 * **Typical usage - tune a PID over a closed-loop simulation:**
 * @code
 *   ctrl::AutoTunerParams atp;
 *   atp.n = 3;  // Kp, Ki, Kd
 *   atp.lower = Eigen::Vector3d(0.0,  0.0, 0.0);
 *   atp.upper = Eigen::Vector3d(10.0, 2.0, 1.0);
 *
 *   ctrl::AutoTuner tuner(atp);
 *   Eigen::Vector3d x0(1.0, 0.1, 0.01);
 *
 *   ctrl::TunerResult result = tuner.tune(
 *       [&](const Eigen::VectorXd &p) -> double {
 *           ctrl::PIDParams pp; pp.Kp = p(0); pp.Ki = p(1); pp.Kd = p(2);
 *           ctrl::DiscretePID pid(pp, Ts);
 *           return simulate_iae(pid, plant, N_steps);
 *       }, x0);
 *
 *   std::cout << "Best IAE = " << result.cost
 *             << "  Kp=" << result.params(0) << "\n";
 * @endcode
 *
 * **Algorithm (Hansen & Ostermeier 2001 - simplified full CMA-ES):**
 * - Population size lambda = 4 + floor(3 ln n).
 * - Log-weighted recombination: mu = lambda/2 best samples.
 * - Cumulative step-length adaptation (CSA) for sigma.
 * - Rank-1 + rank-mu covariance update.
 * - Eigendecomposition of C refreshed every ~1/(c1+cmu)/n/10 generations.
 *
 * @note No external dependencies beyond Eigen and the C++ standard library.
 *
 * @see Hansen, N. & Ostermeier, A. (2001). Completely derandomized self-
 *      adaptation in evolution strategies. Evolutionary Computation 9(2).
 * @see Hansen, N. (2006). The CMA evolution strategy: a tutorial.
 *      arXiv:1604.00772.
 */

namespace ctrl
{

/** @brief Parameters for AutoTuner. */
struct AutoTunerParams
{
    int    n        = 1;     ///< Parameter space dimension.
    double sigma0   = 0.3;   ///< Initial step size (in normalised units if bounds given).
    int    maxIter  = 300;   ///< Maximum generations before stopping.
    double tol      = 1e-10; ///< Converge when max(sigma * D) < tol.
    /**
     * @brief Optional lower bounds for box constraint (size n or empty).
     * Samples outside bounds are reflected back to the boundary.
     */
    Eigen::VectorXd lower;
    /**
     * @brief Optional upper bounds for box constraint (size n or empty).
     * Samples outside bounds are reflected back to the boundary.
     */
    Eigen::VectorXd upper;
};

/** @brief Result of an AutoTuner::tune() run. */
struct TunerResult
{
    Eigen::VectorXd params;    ///< Best parameter vector found.
    double          cost;      ///< Lowest cost achieved.
    int             nEvals;    ///< Total cost function evaluations.
    int             nGens;     ///< Number of CMA-ES generations run.
    bool            converged; ///< @c true if tol criterion was satisfied.
};

/**
 * @brief CMA-ES black-box optimizer.
 *
 * Thread-unsafe (holds PRNG state).  Create one instance per optimization run
 * or protect with a mutex for concurrent use.
 */
class AutoTuner
{
public:
    /** @brief Cost function type: f(params) -> non-negative scalar. */
    using CostFn = std::function<double(const Eigen::VectorXd &)>;

    /**
     * @brief Construct with given parameters and optional random seed.
     * @param params Tuning parameters including n and optional bounds.
     * @param seed   RNG seed for reproducibility (default 42).
     */
    explicit AutoTuner(const AutoTunerParams &params, unsigned seed = 42)
        : p_(params), rng_(seed) {}

    /**
     * @brief Run CMA-ES to minimize cost starting from x0.
     *
     * @param cost User-supplied cost function f(params) -> double.
     * @param x0   Initial parameter vector (n x 1).
     * @return     TunerResult with best params, cost, eval counts, and convergence flag.
     */
    TunerResult tune(CostFn cost, const Eigen::VectorXd &x0);

private:
    Eigen::VectorXd clip(const Eigen::VectorXd &x) const;

    AutoTunerParams p_;
    std::mt19937    rng_;
};

// =============================================================================
// Implementation (header-only)
// =============================================================================

inline Eigen::VectorXd AutoTuner::clip(const Eigen::VectorXd &x) const
{
    if (p_.lower.size() == 0) return x;
    return x.cwiseMax(p_.lower).cwiseMin(p_.upper);
}

inline TunerResult AutoTuner::tune(CostFn cost, const Eigen::VectorXd &x0)
{
    const int n = p_.n;

    // -----------------------------------------------------------------------
    // CMA-ES strategy parameters (Hansen 2006 tutorial Table 1)
    // -----------------------------------------------------------------------
    const int    lambda   = 4 + static_cast<int>(std::floor(3.0 * std::log(static_cast<double>(n))));
    const int    mu       = lambda / 2;

    // Log-weighted recombination weights (normalised)
    Eigen::VectorXd w(mu);
    for (int i = 0; i < mu; ++i)
        w(i) = std::log(static_cast<double>(mu) + 0.5) - std::log(static_cast<double>(i + 1));
    w /= w.sum();
    const double mu_eff = 1.0 / w.squaredNorm(); // variance-effective selection mass

    // Step-size control
    const double c_sigma = (mu_eff + 2.0) / (static_cast<double>(n) + mu_eff + 5.0);
    const double d_sigma = 1.0
        + 2.0 * std::max(0.0, std::sqrt((mu_eff - 1.0) / (static_cast<double>(n) + 1.0)) - 1.0)
        + c_sigma;
    const double chi_n = std::sqrt(static_cast<double>(n))
        * (1.0 - 1.0 / (4.0 * n) + 1.0 / (21.0 * n * n));

    // Covariance adaptation
    const double c_c  = (4.0 + mu_eff / n) / (n + 4.0 + 2.0 * mu_eff / n);
    const double c_1  = 2.0 / ((n + 1.3) * (n + 1.3) + mu_eff);
    const double c_mu = std::min(
        1.0 - c_1,
        2.0 * (mu_eff - 2.0 + 1.0 / mu_eff) / ((n + 2.0) * (n + 2.0) + mu_eff));

    // Eigendecomposition frequency (at most once per generation for small n)
    const int eigen_freq = std::max(
        1, static_cast<int>(std::floor(1.0 / (c_1 + c_mu) / n / 10.0)));

    // -----------------------------------------------------------------------
    // State variables
    // -----------------------------------------------------------------------
    Eigen::VectorXd m       = x0;              // distribution mean
    double          sigma   = p_.sigma0;       // step size
    Eigen::VectorXd p_sigma = Eigen::VectorXd::Zero(n); // CSA path
    Eigen::VectorXd p_c     = Eigen::VectorXd::Zero(n); // covariance path
    Eigen::MatrixXd C       = Eigen::MatrixXd::Identity(n, n); // covariance
    Eigen::MatrixXd B       = Eigen::MatrixXd::Identity(n, n); // eigenvectors of C
    Eigen::VectorXd D       = Eigen::VectorXd::Ones(n);        // sqrt eigenvalues of C

    std::normal_distribution<double> normal(0.0, 1.0);

    // -----------------------------------------------------------------------
    // Tracking
    // -----------------------------------------------------------------------
    double          best_cost   = std::numeric_limits<double>::infinity();
    Eigen::VectorXd best_params = x0;
    int             n_evals     = 0;

    std::vector<Eigen::VectorXd> z_pop(lambda, Eigen::VectorXd(n));
    std::vector<Eigen::VectorXd> y_pop(lambda, Eigen::VectorXd(n));
    std::vector<Eigen::VectorXd> x_pop(lambda, Eigen::VectorXd(n));
    std::vector<double>          f_pop(lambda);
    std::vector<int>             idx(lambda);

    for (int gen = 0; gen < p_.maxIter; ++gen)
    {
        // -------------------------------------------------------------------
        // Sample lambda candidate solutions
        // -------------------------------------------------------------------
        for (int i = 0; i < lambda; ++i)
        {
            for (int j = 0; j < n; ++j) z_pop[i](j) = normal(rng_);
            // y = B * (D .* z)  (covariance-scaled direction)
            y_pop[i] = B * D.cwiseProduct(z_pop[i]);
            x_pop[i] = clip(m + sigma * y_pop[i]);
        }

        // -------------------------------------------------------------------
        // Evaluate cost and track best
        // -------------------------------------------------------------------
        for (int i = 0; i < lambda; ++i)
        {
            f_pop[i] = cost(x_pop[i]);
            ++n_evals;
            if (f_pop[i] < best_cost)
            {
                best_cost   = f_pop[i];
                best_params = x_pop[i];
            }
        }

        // -------------------------------------------------------------------
        // Sort by cost (ascending) to select mu best
        // -------------------------------------------------------------------
        std::iota(idx.begin(), idx.end(), 0);
        std::sort(idx.begin(), idx.end(),
                  [&](int a, int b){ return f_pop[a] < f_pop[b]; });

        // -------------------------------------------------------------------
        // Update mean
        // -------------------------------------------------------------------
        const Eigen::VectorXd m_old = m;
        m.setZero();
        for (int i = 0; i < mu; ++i) m += w(i) * x_pop[idx[i]];
        const Eigen::VectorXd y_w = (m - m_old) / sigma; // mean step direction

        // -------------------------------------------------------------------
        // Update cumulative step-length path p_sigma
        //   p_sigma <-- (1-cs)*p_sigma + sqrt(cs*(2-cs)*mueff) * C^{-1/2} * yw
        // where C^{-1/2} = B * D^{-1} * B^T  (eigenvector decomposition)
        // -------------------------------------------------------------------
        const Eigen::VectorXd Cinv_half_yw =
            B * (D.cwiseInverse().cwiseProduct(B.transpose() * y_w));
        p_sigma = (1.0 - c_sigma) * p_sigma
                + std::sqrt(c_sigma * (2.0 - c_sigma) * mu_eff) * Cinv_half_yw;

        // Step-size update (CSA)
        sigma *= std::exp((c_sigma / d_sigma) * (p_sigma.norm() / chi_n - 1.0));

        // -------------------------------------------------------------------
        // Indicator h_sigma (heaviside for rank-1 update)
        // -------------------------------------------------------------------
        const double ps_norm_eff = p_sigma.norm()
            / std::sqrt(1.0 - std::pow(1.0 - c_sigma, 2.0 * (gen + 1)));
        const double h_threshold = (1.4 + 2.0 / (n + 1.0)) * chi_n;
        const double h_sigma     = (ps_norm_eff < h_threshold) ? 1.0 : 0.0;

        // -------------------------------------------------------------------
        // Update covariance path p_c
        // -------------------------------------------------------------------
        p_c = (1.0 - c_c) * p_c
            + h_sigma * std::sqrt(c_c * (2.0 - c_c) * mu_eff) * y_w;

        // -------------------------------------------------------------------
        // Update covariance matrix C (rank-1 + rank-mu)
        // -------------------------------------------------------------------
        C = (1.0 - c_1 - c_mu) * C
          + c_1 * (p_c * p_c.transpose()
                   + (1.0 - h_sigma) * c_c * (2.0 - c_c) * C);

        for (int i = 0; i < mu; ++i)
        {
            // Use un-clipped y direction for the update for correct statistics
            const Eigen::VectorXd yi = (x_pop[idx[i]] - m_old) / sigma;
            C += c_mu * w(i) * (yi * yi.transpose());
        }

        // -------------------------------------------------------------------
        // Eigendecompose C -> B, D  (every eigen_freq generations)
        // -------------------------------------------------------------------
        if (gen % eigen_freq == 0)
        {
            // Enforce symmetry before decomposition
            C = 0.5 * (C + C.transpose());

            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(C);
            // Floor near-zero eigenvalues for numerical safety
            D = eig.eigenvalues().cwiseMax(1e-20).cwiseSqrt();
            B = eig.eigenvectors();
        }

        // -------------------------------------------------------------------
        // Convergence check: stop when the step-size-scaled spread is tiny
        // -------------------------------------------------------------------
        if ((sigma * D).maxCoeff() < p_.tol)
            return {best_params, best_cost, n_evals, gen + 1, true};
    }

    return {best_params, best_cost, n_evals, p_.maxIter, false};
}

} // namespace ctrl
