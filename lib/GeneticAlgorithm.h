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
 * @file GeneticAlgorithm.h
 * @brief Header-only Genetic Algorithm (GA) for controller parameter tuning.
 *
 * Implements a real-valued GA with:
 * - Tournament selection (size 3)
 * - BLX-alpha crossover (blend-alpha, alpha=0.3 by default)
 * - Gaussian mutation (per-gene, sigma proportional to (upper-lower))
 * - Elitism (best `elite_frac` of population carry forward unchanged)
 *
 * Shares `TunerResult` and `CostFn` with `AutoTuner` and `BayesianOptimizer`.
 *
 * **Usage:**
 * @code
 *   ctrl::GAParams p;
 *   p.n_dim      = 3;
 *   p.lower      = Eigen::Vector3d(0, 0, 0);
 *   p.upper      = Eigen::Vector3d(10, 5, 2);
 *   p.population = 30;
 *   p.max_gen    = 80;
 *
 *   ctrl::GeneticAlgorithm ga(p);
 *   ctrl::TunerResult r = ga.optimize([&](const Eigen::VectorXd& x) {
 *       return simulate_cost(x);
 *   });
 *   // r.params  = best parameter vector
 *   // r.cost    = best cost
 *   // r.nGens   = generations completed
 *   // r.nEvals  = total cost evaluations
 * @endcode
 *
 * @see BayesianOptimizer -- GP surrogate for expensive functions (< 100 evals).
 * @see AutoTuner         -- CMA-ES for cheap cost functions (1000+ evals OK).
 * @see ParticleSwarmOptimizer, DifferentialEvolution -- alternative metaheuristics.
 */

namespace ctrl {

/**
 * @brief Parameters for GeneticAlgorithm.
 */
struct GAParams
{
    int n_dim      = 1;      ///< Search space dimension (set before use).

    int population = 50;     ///< Population size. Must be >= 4.
    int max_gen    = 100;    ///< Maximum number of generations.

    double crossover    = 0.8;   ///< Crossover probability per pair.
    double mutation     = 0.1;   ///< Mutation probability per gene.
    double alpha        = 0.3;   ///< BLX-alpha blend factor (0 = no extrapolation).
    double elite_frac   = 0.05;  ///< Fraction of elites preserved per generation.
    double tol          = 1e-6;  ///< Convergence: stddev of population costs below tol.

    Eigen::VectorXd lower;       ///< Lower bounds (required, size n_dim).
    Eigen::VectorXd upper;       ///< Upper bounds (required, size n_dim).

    unsigned int seed = 42;      ///< RNG seed for reproducibility.
};

/**
 * @brief Real-valued Genetic Algorithm optimiser.
 */
class GeneticAlgorithm
{
public:
    using CostFn = AutoTuner::CostFn;

    explicit GeneticAlgorithm(const GAParams& p = GAParams{}) : p_(p), rng_(p.seed)
    {
        if (p_.population < 4)
            throw std::invalid_argument("GeneticAlgorithm: population must be >= 4");
        if (p_.lower.size() != p_.n_dim || p_.upper.size() != p_.n_dim)
            throw std::invalid_argument("GeneticAlgorithm: bounds size must equal n_dim");
    }

    /**
     * @brief Run the GA to minimise `cost`.
     * @param cost  Objective function f(x) -> non-negative scalar.
     * @return      TunerResult with best params, cost, nEvals, nGens, converged.
     */
    TunerResult optimize(const CostFn& cost)
    {
        const int N  = p_.population;
        const int n  = p_.n_dim;
        const int n_elite = std::max(1, static_cast<int>(std::round(p_.elite_frac * N)));

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

        Eigen::VectorXd best_x = pop[0];
        double best_cost = fitness[0];
        for (int i = 1; i < N; ++i) {
            if (fitness[i] < best_cost) { best_cost = fitness[i]; best_x = pop[i]; }
        }

        bool converged = false;
        int gen = 0;
        for (; gen < p_.max_gen; ++gen) {
            // --- Sort by fitness (ascending = better) ---
            std::vector<int> idx(N);
            std::iota(idx.begin(), idx.end(), 0);
            std::sort(idx.begin(), idx.end(),
                      [&](int a, int b){ return fitness[a] < fitness[b]; });

            std::vector<Eigen::VectorXd> new_pop(N, Eigen::VectorXd(n));
            std::vector<double> new_fit(N);

            // --- Elitism: copy best n_elite unchanged ---
            for (int e = 0; e < n_elite; ++e) {
                new_pop[e] = pop[idx[e]];
                new_fit[e] = fitness[idx[e]];
            }

            // --- Fill remainder via selection + crossover + mutation ---
            for (int child = n_elite; child < N; ++child) {
                // Tournament selection for two parents
                Eigen::VectorXd p1 = tournamentSelect(pop, fitness);
                Eigen::VectorXd p2 = tournamentSelect(pop, fitness);

                Eigen::VectorXd offspring(n);
                std::uniform_real_distribution<double> ud01(0.0, 1.0);
                if (ud01(rng_) < p_.crossover) {
                    // BLX-alpha crossover
                    for (int j = 0; j < n; ++j) {
                        double lo = std::min(p1[j], p2[j]);
                        double hi = std::max(p1[j], p2[j]);
                        double range = hi - lo;
                        std::uniform_real_distribution<double> blx(
                            lo - p_.alpha * range, hi + p_.alpha * range);
                        offspring[j] = blx(rng_);
                    }
                } else {
                    offspring = (ud01(rng_) < 0.5) ? p1 : p2;
                }

                // Gaussian mutation
                for (int j = 0; j < n; ++j) {
                    if (ud01(rng_) < p_.mutation) {
                        double sigma = 0.1 * (p_.upper[j] - p_.lower[j]);
                        std::normal_distribution<double> nd(0.0, sigma);
                        offspring[j] += nd(rng_);
                    }
                    // Clip to bounds
                    offspring[j] = std::max(p_.lower[j], std::min(p_.upper[j], offspring[j]));
                }

                new_pop[child] = offspring;
                new_fit[child] = cost(offspring);
                ++n_evals;
                if (new_fit[child] < best_cost) {
                    best_cost = new_fit[child];
                    best_x    = new_pop[child];
                }
            }

            pop     = std::move(new_pop);
            fitness = std::move(new_fit);

            // --- Convergence check: stddev of fitness ---
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
    GAParams       p_;
    std::mt19937   rng_;

    // Tournament selection (size 3) - returns copy of winner individual
    Eigen::VectorXd tournamentSelect(const std::vector<Eigen::VectorXd>& pop,
                                     const std::vector<double>& fitness)
    {
        const int N = static_cast<int>(pop.size());
        std::uniform_int_distribution<int> uid(0, N - 1);
        int best = uid(rng_);
        for (int t = 1; t < 3; ++t) {
            int cand = uid(rng_);
            if (fitness[cand] < fitness[best]) best = cand;
        }
        return pop[best];
    }
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(genetic_algorithm)
