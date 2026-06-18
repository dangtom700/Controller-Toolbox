#pragma once
#include "AutoTuner.h"          // TunerResult, CostFn
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
 * @file ParticleSwarmOptimizer.h
 * @brief Header-only Particle Swarm Optimization (PSO) for controller parameter tuning.
 *
 * Implements the standard PSO with Clerc-Kennedy constriction coefficients:
 * - w  = 0.729  (inertia weight)
 * - c1 = 1.495  (cognitive / personal best)
 * - c2 = 1.495  (social / global best)
 * - Velocity clamping: V_max = 0.2 * (upper - lower) per dimension
 *
 * Shares `TunerResult` and `CostFn` with `AutoTuner` and `BayesianOptimizer`.
 *
 * **Usage:**
 * @code
 *   ctrl::PSOParams p;
 *   p.n_dim       = 3;
 *   p.lower       = Eigen::Vector3d(0, 0, 0);
 *   p.upper       = Eigen::Vector3d(10, 5, 2);
 *   p.n_particles = 20;
 *   p.max_iter    = 80;
 *
 *   ctrl::ParticleSwarmOptimizer pso(p);
 *   ctrl::TunerResult r = pso.optimize([&](const Eigen::VectorXd& x) {
 *       return simulate_cost(x);
 *   });
 * @endcode
 *
 * @see GeneticAlgorithm, DifferentialEvolution -- alternative metaheuristics.
 * @see Clerc & Kennedy (2002). The particle swarm - explosion, stability, and
 *      convergence in a multidimensional complex space. IEEE TEC 6(1):58-73.
 */

namespace ctrl {

/**
 * @brief Parameters for ParticleSwarmOptimizer.
 */
struct PSOParams
{
    int n_dim       = 1;     ///< Search space dimension (set before use).

    int n_particles = 30;    ///< Swarm size.
    int max_iter    = 100;   ///< Maximum iterations.

    double w        = 0.729;  ///< Inertia weight (Clerc-Kennedy default).
    double c1       = 1.495;  ///< Cognitive coefficient (personal best).
    double c2       = 1.495;  ///< Social coefficient (global best).
    double v_frac   = 0.2;    ///< V_max as fraction of (upper - lower).
    double tol      = 1e-6;   ///< Convergence: swarm spread (std of costs) below tol.

    Eigen::VectorXd lower;    ///< Lower bounds (required, size n_dim).
    Eigen::VectorXd upper;    ///< Upper bounds (required, size n_dim).

    unsigned int seed = 42;   ///< RNG seed.
};

/**
 * @brief Particle Swarm Optimizer.
 */
class ParticleSwarmOptimizer
{
public:
    using CostFn = AutoTuner::CostFn;

    explicit ParticleSwarmOptimizer(const PSOParams& p = PSOParams{}) : p_(p), rng_(p.seed)
    {
        if (p_.n_particles < 2)
            throw std::invalid_argument("ParticleSwarmOptimizer: n_particles must be >= 2");
        if (p_.lower.size() != p_.n_dim || p_.upper.size() != p_.n_dim)
            throw std::invalid_argument("ParticleSwarmOptimizer: bounds size must equal n_dim");
    }

    /**
     * @brief Run PSO to minimise `cost`.
     * @return TunerResult with best params, cost, nEvals, nGens (= iterations), converged.
     */
    TunerResult optimize(const CostFn& cost)
    {
        const int N = p_.n_particles;
        const int n = p_.n_dim;

        // V_max per dimension
        Eigen::VectorXd v_max = p_.v_frac * (p_.upper - p_.lower);

        // --- Initialise positions and velocities ---
        std::vector<Eigen::VectorXd> pos(N, Eigen::VectorXd(n));
        std::vector<Eigen::VectorXd> vel(N, Eigen::VectorXd(n));
        std::vector<Eigen::VectorXd> pbest(N, Eigen::VectorXd(n));
        std::vector<double> pbest_cost(N, std::numeric_limits<double>::max());

        std::uniform_real_distribution<double> ud01(0.0, 1.0);

        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < n; ++j) {
                std::uniform_real_distribution<double> udx(p_.lower[j], p_.upper[j]);
                pos[i][j] = udx(rng_);
                std::uniform_real_distribution<double> udv(-v_max[j], v_max[j]);
                vel[i][j] = udv(rng_);
            }
            pbest_cost[i] = cost(pos[i]);
            pbest[i]      = pos[i];
        }
        int n_evals = N;

        // Global best
        Eigen::VectorXd gbest(n);
        double gbest_cost = std::numeric_limits<double>::max();
        for (int i = 0; i < N; ++i) {
            if (pbest_cost[i] < gbest_cost) {
                gbest_cost = pbest_cost[i];
                gbest      = pbest[i];
            }
        }

        bool converged = false;
        int iter = 0;
        for (; iter < p_.max_iter; ++iter) {
            for (int i = 0; i < N; ++i) {
                for (int j = 0; j < n; ++j) {
                    double r1 = ud01(rng_);
                    double r2 = ud01(rng_);
                    vel[i][j] = p_.w  * vel[i][j]
                               + p_.c1 * r1 * (pbest[i][j] - pos[i][j])
                               + p_.c2 * r2 * (gbest[j]    - pos[i][j]);
                    // Velocity clamping
                    vel[i][j] = std::max(-v_max[j], std::min(v_max[j], vel[i][j]));

                    pos[i][j] += vel[i][j];
                    // Position clamping + zero velocity on boundary hit
                    if (pos[i][j] < p_.lower[j]) { pos[i][j] = p_.lower[j]; vel[i][j] = 0.0; }
                    if (pos[i][j] > p_.upper[j]) { pos[i][j] = p_.upper[j]; vel[i][j] = 0.0; }
                }

                double c = cost(pos[i]);
                ++n_evals;
                if (c < pbest_cost[i]) {
                    pbest_cost[i] = c;
                    pbest[i]      = pos[i];
                    if (c < gbest_cost) { gbest_cost = c; gbest = pos[i]; }
                }
            }

            // Convergence: stddev of personal best costs
            double mean = 0.0;
            for (double c : pbest_cost) mean += c;
            mean /= N;
            double var = 0.0;
            for (double c : pbest_cost) var += (c - mean) * (c - mean);
            if (std::sqrt(var / N) < p_.tol) { converged = true; break; }
        }

        TunerResult r;
        r.params    = gbest;
        r.cost      = gbest_cost;
        r.nEvals    = n_evals;
        r.nGens     = iter + 1;
        r.converged = converged;
        return r;
    }

private:
    PSOParams     p_;
    std::mt19937  rng_;
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(particle_swarm)
