#pragma once
#include "AutoTuner.h"          // TunerResult, CostFn
#include "ControllerRegistry.h"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>
#include <random>
#include <stdexcept>
#include <vector>

/**
 * @file DifferentialEvolution.h
 * @brief Header-only Differential Evolution (DE) for controller parameter tuning.
 *
 * Implements DE/rand/1/bin:
 * - Mutation:  v = x_r1 + F * (x_r2 - x_r3)  (three random distinct parents)
 * - Crossover: binomial (per-gene coin flip with probability CR)
 * - Selection: greedy (trial vector replaces target if cost is lower)
 * - Boundary:  reflection for out-of-bounds mutants
 *
 * Shares `TunerResult` and `CostFn` with `AutoTuner` and `BayesianOptimizer`.
 *
 * **Usage:**
 * @code
 *   ctrl::DEParams p;
 *   p.n_dim    = 3;
 *   p.lower    = Eigen::Vector3d(0, 0, 0);
 *   p.upper    = Eigen::Vector3d(10, 5, 2);
 *   p.population = 20;
 *   p.max_gen    = 80;
 *   p.F  = 0.8;   // mutation factor
 *   p.CR = 0.9;   // crossover rate
 *
 *   ctrl::DifferentialEvolution de(p);
 *   ctrl::TunerResult r = de.optimize([&](const Eigen::VectorXd& x) {
 *       return simulate_cost(x);
 *   });
 * @endcode
 *
 * @see GeneticAlgorithm, ParticleSwarmOptimizer -- alternative metaheuristics.
 * @see Storn & Price (1997). Differential Evolution -- a simple and efficient
 *      heuristic for global optimization over continuous spaces. J. Global Opt. 11:341-359.
 */

namespace ctrl {

/**
 * @brief Parameters for DifferentialEvolution.
 */
struct DEParams
{
    int n_dim      = 1;      ///< Search space dimension (set before use).

    int population = 30;     ///< Population size. Must be >= 4.
    int max_gen    = 100;    ///< Maximum number of generations.

    double F    = 0.8;       ///< Mutation factor in [0.5, 1.0]. Controls differential step.
    double CR   = 0.9;       ///< Crossover rate in [0, 1]. Higher = more mixing.
    double tol  = 1e-6;      ///< Convergence: stddev of population costs below tol.

    Eigen::VectorXd lower;   ///< Lower bounds (required, size n_dim).
    Eigen::VectorXd upper;   ///< Upper bounds (required, size n_dim).

    unsigned int seed = 42;  ///< RNG seed.
};

/**
 * @brief Differential Evolution optimiser.
 */
class DifferentialEvolution
{
public:
    using CostFn = AutoTuner::CostFn;

    explicit DifferentialEvolution(const DEParams& p = DEParams{}) : p_(p), rng_(p.seed)
    {
        if (p_.population < 4)
            throw std::invalid_argument("DifferentialEvolution: population must be >= 4");
        if (p_.lower.size() != p_.n_dim || p_.upper.size() != p_.n_dim)
            throw std::invalid_argument("DifferentialEvolution: bounds size must equal n_dim");
    }

    /**
     * @brief Run DE to minimise `cost`.
     * @return TunerResult with best params, cost, nEvals, nGens, converged.
     */
    TunerResult optimize(const CostFn& cost)
    {
        const int N = p_.population;
        const int n = p_.n_dim;

        // --- Initialise population uniformly in [lower, upper] ---
        std::vector<Eigen::VectorXd> pop(N, Eigen::VectorXd(n));
        std::vector<double> fitness(N);

        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < n; ++j) {
                std::uniform_real_distribution<double> ud(p_.lower[j], p_.upper[j]);
                pop[i][j] = ud(rng_);
            }
            fitness[i] = cost(pop[i]);
        }
        int n_evals = N;

        int best_idx = static_cast<int>(
            std::min_element(fitness.begin(), fitness.end()) - fitness.begin());
        Eigen::VectorXd best_x    = pop[best_idx];
        double           best_cost = fitness[best_idx];

        bool converged = false;
        int gen = 0;

        std::uniform_int_distribution<int> uid(0, N - 1);
        std::uniform_real_distribution<double> ud01(0.0, 1.0);
        std::uniform_int_distribution<int> uid_dim(0, n - 1);

        for (; gen < p_.max_gen; ++gen) {
            for (int i = 0; i < N; ++i) {
                // Pick 3 distinct random indices != i
                int r1, r2, r3;
                do { r1 = uid(rng_); } while (r1 == i);
                do { r2 = uid(rng_); } while (r2 == i || r2 == r1);
                do { r3 = uid(rng_); } while (r3 == i || r3 == r1 || r3 == r2);

                // Mutation: DE/rand/1
                Eigen::VectorXd v = pop[r1] + p_.F * (pop[r2] - pop[r3]);

                // Boundary handling: reflect out-of-bounds genes (capped to avoid infinite loop
                // when lower==upper or a large mutation overshoots both bounds simultaneously)
                for (int j = 0; j < n; ++j) {
                    int reflect_iters = 0;
                    while ((v[j] < p_.lower[j] || v[j] > p_.upper[j]) && ++reflect_iters < 20) {
                        if (v[j] < p_.lower[j]) v[j] = 2.0 * p_.lower[j] - v[j];
                        if (v[j] > p_.upper[j]) v[j] = 2.0 * p_.upper[j] - v[j];
                    }
                    v[j] = std::clamp(v[j], p_.lower[j], p_.upper[j]);
                }

                // Crossover: binomial (at least one gene from mutant)
                Eigen::VectorXd trial = pop[i];
                int j_rand = uid_dim(rng_);
                for (int j = 0; j < n; ++j) {
                    if (j == j_rand || ud01(rng_) < p_.CR)
                        trial[j] = v[j];
                }

                // Greedy selection
                double trial_cost = cost(trial);
                ++n_evals;
                if (trial_cost <= fitness[i]) {
                    pop[i]     = trial;
                    fitness[i] = trial_cost;
                    if (trial_cost < best_cost) {
                        best_cost = trial_cost;
                        best_x    = trial;
                    }
                }
            }

            // Convergence: stddev of population fitness
            double mean = 0.0;
            for (double f : fitness) mean += f;
            mean /= N;
            double var = 0.0;
            for (double f : fitness) var += (f - mean) * (f - mean);
            if (std::sqrt(var / N) < p_.tol) { converged = true; break; }
        }

        TunerResult r;
        r.params    = best_x;
        r.cost      = best_cost;
        r.nEvals    = n_evals;
        r.nGens     = gen + 1;
        r.converged = converged;
        return r;
    }

private:
    DEParams      p_;
    std::mt19937  rng_;
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(differential_evolution)
