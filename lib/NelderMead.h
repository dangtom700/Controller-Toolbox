#pragma once
#include "AutoTuner.h"          // TunerResult, CostFn
#include "ControllerRegistry.h"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <random>
#include <stdexcept>
#include <vector>

/**
 * @file NelderMead.h
 * @brief Header-only Nelder-Mead simplex optimizer (Phase 3 MO2).
 *
 * Derivative-free, non-population direct-search optimizer (reflect/expand/contract/
 * shrink) - a lighter-weight alternative to CMA-ES/GA/PSO/DE for low-dimensional
 * (n < 10) quick tuning where population overhead isn't justified: it needs only an
 * initial point, no bounds, no population sizing.
 *
 * Shares `TunerResult`/`CostFn` with `AutoTuner` and drops into the exact same call
 * site as `GeneticAlgorithm`/`ParticleSwarmOptimizer`/`DifferentialEvolution`.
 *
 * **Algorithm (Nelder & Mead 1965; Lagarias et al. 1998 conventions):** maintain
 * `n_dim + 1` simplex vertices sorted by cost ascending. Each iteration: reflect the
 * worst vertex through the centroid of the rest; expand further if the reflection
 * beat the best point; otherwise contract (outside/inside) toward the centroid; if
 * contraction doesn't improve, shrink the whole simplex toward the best vertex. If
 * the simplex collapses (max edge length below a numerical floor) before the cost
 * spread has actually converged, restart once from the best vertex with a fresh
 * perturbed simplex rather than silently returning a degenerate result.
 *
 * @see Nelder, J.A. & Mead, R. (1965). A simplex method for function minimization.
 *      Computer Journal, 7(4), 308-313.
 * @see Lagarias, J.C. et al. (1998). Convergence properties of the Nelder-Mead
 *      simplex method in low dimensions. SIAM J. Optim. 9(1), 112-147.
 */

namespace ctrl {

/** @brief Parameters for NelderMead. */
struct NelderMeadParams
{
    int    n_dim    = 1;     ///< Search space dimension.
    int    max_iter = 500;   ///< Maximum iterations before stopping.
    double tol      = 1e-8;  ///< Converge when both the cost spread and simplex spread are below this.
    double alpha    = 1.0;   ///< Reflection coefficient.
    double gamma    = 2.0;   ///< Expansion coefficient.
    double rho      = 0.5;   ///< Contraction coefficient.
    double sigma    = 0.5;   ///< Shrink coefficient.
    unsigned seed   = 42;    ///< RNG seed for the initial-simplex perturbation.
};

/**
 * @brief Nelder-Mead simplex optimizer.
 */
class NelderMead
{
public:
    /** @brief Cost function type, shared with AutoTuner/GeneticAlgorithm/etc. */
    using CostFn = AutoTuner::CostFn;

    explicit NelderMead(const NelderMeadParams &p) : p_(p), rng_(p.seed)
    {
        if (p_.n_dim < 1)
            throw std::invalid_argument("NelderMead: n_dim must be >= 1");
    }

    /**
     * @brief Run Nelder-Mead to minimize `cost` starting from `x0`.
     * @param cost Objective function f(x) -> non-negative scalar.
     * @param x0   Initial point (n_dim x 1).
     * @return     TunerResult with best params, cost, eval counts, and convergence flag.
     */
    TunerResult optimize(const CostFn &cost, const Eigen::VectorXd &x0)
    {
        const int n = p_.n_dim;
        int n_evals = 0;
        bool restarted = false;

        std::vector<Eigen::VectorXd> simplex;
        std::vector<double> fval;
        buildInitialSimplex(x0, simplex);
        fval.resize(n + 1);
        for (int i = 0; i <= n; ++i) { fval[i] = cost(simplex[i]); ++n_evals; }

        bool converged = false;
        int iter = 0;
        for (; iter < p_.max_iter; ++iter)
        {
            sortByCost(simplex, fval);

            const double fspread = fval[n] - fval[0];
            double xspread = 0.0;
            for (int i = 1; i <= n; ++i)
                xspread = std::max(xspread, (simplex[i] - simplex[0]).cwiseAbs().maxCoeff());

            // Collapse detection (checked *before* the regular convergence test below):
            // the simplex has degenerated to a near-zero-size cluster - e.g. a flat-cost
            // plateau, where every reflect/contract step sees an unchanging cost and the
            // shrink step is all that's left, geometrically halving the simplex every
            // iteration without ever resolving a real minimum. kCollapseEps sits a few
            // orders of magnitude above `tol` so a *genuinely* converging search (where
            // xspread and fspread shrink together) is caught by the regular check first,
            // while a degenerate collapse - tiny xspread, but fspread not yet near tol -
            // is caught here instead. Restart once from the best vertex, using a
            // *randomized* (not axis-aligned) perturbation basis so the restart isn't
            // deterministically retracing the same degenerate directions.
            const double kCollapseEps = std::max(100.0 * p_.tol, 1e-10);
            if (xspread < kCollapseEps && !(fspread < p_.tol) && !restarted)
            {
                restarted = true;
                buildRandomSimplex(simplex[0], simplex);
                for (int i = 0; i <= n; ++i) { fval[i] = cost(simplex[i]); ++n_evals; }
                continue;
            }

            if (fspread < p_.tol && xspread < p_.tol)
            {
                converged = true;
                break;
            }

            Eigen::VectorXd centroid = Eigen::VectorXd::Zero(n);
            for (int i = 0; i < n; ++i) centroid += simplex[i];
            centroid /= static_cast<double>(n);

            const Eigen::VectorXd xr = centroid + p_.alpha * (centroid - simplex[n]);
            const double fr = cost(xr); ++n_evals;

            if (fr < fval[0])
            {
                // Expansion
                const Eigen::VectorXd xe = centroid + p_.gamma * (xr - centroid);
                const double fe = cost(xe); ++n_evals;
                if (fe < fr) { simplex[n] = xe; fval[n] = fe; }
                else         { simplex[n] = xr; fval[n] = fr; }
            }
            else if (fr < fval[n - 1])
            {
                // Accept reflection
                simplex[n] = xr; fval[n] = fr;
            }
            else
            {
                bool didContract = false;
                if (fr < fval[n])
                {
                    // Outside contraction
                    const Eigen::VectorXd xc = centroid + p_.rho * (xr - centroid);
                    const double fc = cost(xc); ++n_evals;
                    if (fc <= fr) { simplex[n] = xc; fval[n] = fc; didContract = true; }
                }
                else
                {
                    // Inside contraction
                    const Eigen::VectorXd xcc = centroid + p_.rho * (simplex[n] - centroid);
                    const double fcc = cost(xcc); ++n_evals;
                    if (fcc < fval[n]) { simplex[n] = xcc; fval[n] = fcc; didContract = true; }
                }

                if (!didContract)
                {
                    // Shrink toward the best vertex
                    for (int i = 1; i <= n; ++i)
                    {
                        simplex[i] = simplex[0] + p_.sigma * (simplex[i] - simplex[0]);
                        fval[i]    = cost(simplex[i]); ++n_evals;
                    }
                }
            }
        }

        sortByCost(simplex, fval);
        return TunerResult{simplex[0], fval[0], n_evals, iter + 1, converged};
    }

private:
    void buildInitialSimplex(const Eigen::VectorXd &x0, std::vector<Eigen::VectorXd> &simplex) const
    {
        const int n = p_.n_dim;
        simplex.assign(n + 1, x0);
        for (int i = 0; i < n; ++i)
        {
            const double step = std::max(0.05 * std::fabs(x0(i)), 1e-4);
            simplex[i + 1](i) += step;
        }
    }

    // Restart-after-collapse simplex: same per-axis step magnitude as buildInitialSimplex,
    // but each vertex is displaced along a random direction (not a single coordinate axis)
    // so a restart isn't deterministically retracing the directions that just collapsed.
    void buildRandomSimplex(const Eigen::VectorXd &x0, std::vector<Eigen::VectorXd> &simplex)
    {
        const int n = p_.n_dim;
        const double avgStep = [&] {
            double s = 0.0;
            for (int i = 0; i < n; ++i) s += std::max(0.05 * std::fabs(x0(i)), 1e-4);
            return s / n;
        }();

        std::normal_distribution<double> normal(0.0, 1.0);
        simplex.assign(n + 1, x0);
        for (int i = 1; i <= n; ++i)
        {
            Eigen::VectorXd dir(n);
            for (int j = 0; j < n; ++j) dir(j) = normal(rng_);
            const double norm = dir.norm();
            if (norm > 1e-300) dir /= norm;
            simplex[i] = x0 + avgStep * dir;
        }
    }

    static void sortByCost(std::vector<Eigen::VectorXd> &simplex, std::vector<double> &fval)
    {
        const int n1 = static_cast<int>(fval.size());
        std::vector<int> idx(n1);
        std::iota(idx.begin(), idx.end(), 0);
        std::sort(idx.begin(), idx.end(), [&](int a, int b) { return fval[a] < fval[b]; });

        std::vector<Eigen::VectorXd> s2(n1);
        std::vector<double> f2(n1);
        for (int i = 0; i < n1; ++i) { s2[i] = simplex[idx[i]]; f2[i] = fval[idx[i]]; }
        simplex = std::move(s2);
        fval    = std::move(f2);
    }

    NelderMeadParams p_;
    std::mt19937     rng_; ///< Drives buildRandomSimplex()'s restart-after-collapse perturbation.
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(nelder_mead)
