#pragma once
#include "AutoTuner.h"
#include "ControllerRegistry.h"
#include <Eigen/Dense>
#include <functional>

/**
 * @file ConstrainedTuning.h
 * @brief General nonlinear-constraint tuning via an exterior-penalty wrapper (Phase 3 Roadmap
 *        Phase 2 MO3).
 *
 * Extends any existing `CostFn`-based optimizer (AutoTuner, GeneticAlgorithm,
 * ParticleSwarmOptimizer, DifferentialEvolution, NelderMead) to general nonlinear inequality
 * constraints `g(theta) <= 0`, via an exterior (quadratic) penalty method - a pure wrapper, no
 * change needed inside any optimizer (penalty methods only transform the cost function passed in).
 *
 * @see docs/superpowers/specs/2026-06-25-optimization-extensions-design.md
 */

namespace ctrl
{

/** @brief Parameters for tuneConstrained. */
struct ConstrainedTuneParams
{
    std::function<Eigen::VectorXd(const Eigen::VectorXd &)> constraints; ///< Feasible iff all <= 0.
    double penalty_init = 10.0;
    double penalty_growth = 10.0;
    int outer_iters = 5;
    double feasTol = 1e-4; ///< constraints(x) <= feasTol counts as feasible for `converged`.
};

/**
 * @brief Minimize `objective` subject to `params.constraints(theta) <= 0` via an exterior
 *        quadratic penalty method, using any existing optimizer through `optimizerRun`.
 *
 * @param optimizerRun Adapts any optimizer's call shape to (CostFn, x0) -> TunerResult, e.g.
 *        `[&](const AutoTuner::CostFn& c, const Eigen::VectorXd& x0){ return tuner.tune(c, x0); }`
 *        or `[&](const AutoTuner::CostFn& c, const Eigen::VectorXd&){ return ga.optimize(c); }`.
 * @param objective The true (unpenalized) objective to minimize.
 * @param params Penalty/outer-loop configuration.
 * @param x0 Initial parameter vector.
 * @return TunerResult with the true (not penalized) objective's cost at the final point;
 *         `converged` is true iff the last inner optimization converged AND the result is
 *         feasible within `feasTol`.
 */
TunerResult tuneConstrained(
    std::function<TunerResult(const AutoTuner::CostFn &, const Eigen::VectorXd &)> optimizerRun,
    const AutoTuner::CostFn &objective, const ConstrainedTuneParams &params,
    const Eigen::VectorXd &x0);

} // namespace ctrl

CTRL_REGISTER_FEATURE(constrained_tuning)
