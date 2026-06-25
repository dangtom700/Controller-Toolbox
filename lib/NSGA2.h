#pragma once
#include "ControllerRegistry.h"
#include <Eigen/Dense>
#include <functional>
#include <random>

/**
 * @file NSGA2.h
 * @brief NSGA-II multi-objective (Pareto) optimization (Phase 3 Roadmap Phase 2 MO1).
 *
 * Real-valued NSGA-II (Deb, Pratap, Agarwal & Meyarivan, IEEE TEC 2002): fast non-dominated
 * sorting + crowding-distance elitist replacement, returning a Pareto front rather than a
 * single best point. Every other metaheuristic in `lib/` (AutoTuner, GeneticAlgorithm,
 * ParticleSwarmOptimizer, DifferentialEvolution) is single-objective.
 *
 * @see docs/superpowers/specs/2026-06-25-optimization-extensions-design.md
 */

namespace ctrl
{

/** @brief Parameters for NSGA2. */
struct NSGA2Params
{
    int n_dim = 1;
    int n_objectives = 2;
    int population = 100;
    int max_gen = 200;
    double crossover = 0.9; ///< BLX-alpha crossover probability per pair.
    double mutation = 0.1;  ///< Per-gene Gaussian mutation probability.
    double alpha = 0.3;     ///< BLX-alpha blend factor.
    Eigen::VectorXd lower;  ///< Required, size n_dim.
    Eigen::VectorXd upper;  ///< Required, size n_dim.
    unsigned seed = 42;
};

/** @brief Result of NSGA2::optimize - the final population's rank-0 (non-dominated) front. */
struct ParetoResult
{
    Eigen::MatrixXd front_params;     ///< One row per non-dominated solution.
    Eigen::MatrixXd front_objectives; ///< Corresponding objective vectors.
    int nGens = 0;
    int nEvals = 0;
};

/** @brief Multi-objective cost: f(params) -> objective vector, minimize every component. */
using MultiCostFn = std::function<Eigen::VectorXd(const Eigen::VectorXd &)>;

/**
 * @brief NSGA-II multi-objective evolutionary optimizer.
 */
class NSGA2
{
public:
    explicit NSGA2(const NSGA2Params &p);

    /** @brief Run NSGA-II to find the Pareto front minimizing every component of cost(). */
    ParetoResult optimize(const MultiCostFn &cost);

private:
    NSGA2Params p_;
    std::mt19937 rng_;
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(nsga2)
