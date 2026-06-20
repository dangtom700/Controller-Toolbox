#pragma once
#include "AutoTuner.h"
#include "RobustnessAnalysis.h"
#include <Eigen/Dense>
#include <cmath>
#include <cstdint>
#include <functional>
#include <stdexcept>

/**
 * @file WorstCaseSearch.h
 * @brief CMA-ES worst-case parameter search over plant uncertainty.
 *
 * Treats the uncertain plant parameters as the optimisation variables and uses
 * @ref AutoTuner (CMA-ES) to find the parameter vector that *maximises* a robustness
 * metric (peak sensitivity, step IAE, or a user-supplied function) - i.e. the point
 * inside the uncertainty box that degrades closed-loop performance the most.
 *
 * The search runs in normalised coordinates: physical parameter @c i is
 * `param_nominal(i) + z(i) * search_width(i)`, where
 * `search_width(i) = param_sigma(i) * max(|param_nominal(i)|, 1)`. This lets a single
 * scalar CMA-ES step size (`WorstCaseSearchParams::sigma_init`) explore every parameter
 * at its own relative scale, and lets box bounds be supplied in physical units.
 *
 * Phase 4 of the robustness-analysis roadmap (docs/robust_implementation_plan.md).
 *
 * @see RobustnessAnalysis.h, AutoTuner.h
 */

namespace ctrl
{

/** @brief Parameters controlling the CMA-ES worst-case search. */
struct WorstCaseSearchParams
{
    int      max_evals  = 500;  ///< Approximate cost-function evaluation budget.
    double   sigma_init = 0.1;  ///< Initial CMA-ES step size, in relative (search_width) units.
    /**
     * @brief Reserved for a future custom CMA-ES population size.
     *
     * @c AutoTuner derives its population size internally from the parameter
     * dimension (`4 + floor(3*ln(n))`, Hansen 2006) and does not currently expose an
     * override, so this field has no effect. Kept for forward API compatibility with
     * the original design (0 = CMA-ES default).
     */
    int      population = 0;
    uint32_t seed       = 42;   ///< RNG seed forwarded to AutoTuner.
};

/** @brief Result of a worst-case parameter search. */
struct WorstCaseResult
{
    Eigen::VectorXd worst_params; ///< Parameter vector (physical units) that achieved worst_cost.
    double          worst_cost  = 0.0;   ///< Metric value at worst_params (the actual metric, not negated).
    bool            converged   = false; ///< CMA-ES internal convergence flag.
    int             n_evals     = 0;     ///< Total cost-function evaluations used.
};

namespace detail
{

/**
 * @brief Shared CMA-ES driver: maximise @p metric_of_params over a relative search box.
 *
 * @c metric_of_params(phys_params) must return the metric to MAXIMISE (higher = worse);
 * internally this minimises the negation via AutoTuner.
 */
inline WorstCaseResult runWorstCaseSearch(
    const std::function<double(const Eigen::VectorXd&)>& metric_of_params,
    const Eigen::VectorXd& param_nominal,
    const Eigen::VectorXd& param_sigma,
    const Eigen::VectorXd& lower_bounds,
    const Eigen::VectorXd& upper_bounds,
    const WorstCaseSearchParams& p)
{
    const int n = static_cast<int>(param_nominal.size());
    if (n == 0)
        throw std::invalid_argument("findWorstCase*: param_nominal must be non-empty.");
    if (param_sigma.size() != n)
        throw std::invalid_argument("findWorstCase*: param_sigma must match param_nominal size.");
    if (lower_bounds.size() != 0 && lower_bounds.size() != n)
        throw std::invalid_argument("findWorstCase*: lower_bounds must be empty or match param_nominal size.");
    if (upper_bounds.size() != 0 && upper_bounds.size() != n)
        throw std::invalid_argument("findWorstCase*: upper_bounds must be empty or match param_nominal size.");

    // Per-parameter relative search width, in physical units.
    Eigen::VectorXd search_width(n);
    for (int i = 0; i < n; ++i)
    {
        const double scale = std::max(std::abs(param_nominal(i)), 1.0);
        search_width(i) = std::max(param_sigma(i) * scale, 1e-12);
    }

    AutoTunerParams atp;
    atp.n        = n;
    atp.sigma0   = p.sigma_init;
    const int lambda = 4 + static_cast<int>(std::floor(3.0 * std::log(static_cast<double>(std::max(n, 1)))));
    atp.maxIter  = std::max(1, p.max_evals / std::max(lambda, 1));

    if (lower_bounds.size() == n)
    {
        atp.lower.resize(n);
        for (int i = 0; i < n; ++i) atp.lower(i) = (lower_bounds(i) - param_nominal(i)) / search_width(i);
    }
    if (upper_bounds.size() == n)
    {
        atp.upper.resize(n);
        for (int i = 0; i < n; ++i) atp.upper(i) = (upper_bounds(i) - param_nominal(i)) / search_width(i);
    }

    AutoTuner tuner(atp, p.seed);

    auto cost_fn = [&](const Eigen::VectorXd& z) -> double
    {
        const Eigen::VectorXd phys = param_nominal + z.cwiseProduct(search_width);
        return -metric_of_params(phys);
    };

    const TunerResult tr = tuner.tune(cost_fn, Eigen::VectorXd::Zero(n));

    WorstCaseResult result;
    result.worst_params = param_nominal + tr.params.cwiseProduct(search_width);
    result.worst_cost    = -tr.cost;
    result.converged     = tr.converged;
    result.n_evals       = tr.nEvals;
    return result;
}

} // namespace detail

/**
 * @brief Find the uncertain plant that maximises closed-loop peak sensitivity ||S||_inf.
 *
 * @param plant_factory Maps a physical parameter vector to a StateSpace plant.
 * @param controller_ss Fixed, pre-designed controller (error -> input).
 * @param param_nominal Nominal parameter vector (the centre of the search).
 * @param param_sigma   Per-parameter relative search width: search_width(i) =
 *                       param_sigma(i) * max(|param_nominal(i)|, 1) (see file header).
 * @param lower_bounds  Optional hard lower bounds (physical units; empty = unbounded).
 * @param upper_bounds  Optional hard upper bounds (physical units; empty = unbounded).
 * @param p             Search parameters (evaluation budget, initial step, seed).
 * @return WorstCaseResult with the worst-case parameter vector and its ||S||_inf.
 */
inline WorstCaseResult findWorstCaseSensitivity(
    std::function<StateSpace(const Eigen::VectorXd&)> plant_factory,
    const StateSpace& controller_ss,
    const Eigen::VectorXd& param_nominal,
    const Eigen::VectorXd& param_sigma,
    const Eigen::VectorXd& lower_bounds = {},
    const Eigen::VectorXd& upper_bounds = {},
    const WorstCaseSearchParams& p = {})
{
    const StateSpace nominal_plant = plant_factory(param_nominal);
    auto metric = [&](const Eigen::VectorXd& phys) -> double
    {
        const StateSpace plant = plant_factory(phys);
        const RobustnessSample s = evaluateSample(0, plant, controller_ss, nominal_plant);
        return s.hinf_sensitivity;
    };
    return detail::runWorstCaseSearch(metric, param_nominal, param_sigma, lower_bounds, upper_bounds, p);
}

/**
 * @brief Find the uncertain plant that maximises closed-loop step-response IAE.
 *
 * Same search mechanics as @ref findWorstCaseSensitivity; an unstable sample reports
 * @c iae = +inf (see RobustnessAnalysis.h), which CMA-ES correctly treats as the
 * worst possible outcome.
 *
 * @param sim_duration_s Simulation horizon for the IAE evaluation [s].
 */
inline WorstCaseResult findWorstCaseIAE(
    std::function<StateSpace(const Eigen::VectorXd&)> plant_factory,
    const StateSpace& controller_ss,
    const Eigen::VectorXd& param_nominal,
    const Eigen::VectorXd& param_sigma,
    const Eigen::VectorXd& lower_bounds = {},
    const Eigen::VectorXd& upper_bounds = {},
    double sim_duration_s = 50.0,
    const WorstCaseSearchParams& p = {})
{
    const StateSpace nominal_plant = plant_factory(param_nominal);
    auto metric = [&](const Eigen::VectorXd& phys) -> double
    {
        const StateSpace plant = plant_factory(phys);
        const RobustnessSample s = evaluateSample(0, plant, controller_ss, nominal_plant,
                                                   /*step_amplitude=*/1.0, sim_duration_s);
        return s.iae;
    };
    return detail::runWorstCaseSearch(metric, param_nominal, param_sigma, lower_bounds, upper_bounds, p);
}

/**
 * @brief Generic worst-case search: maximise a user-supplied metric of the spawned plant.
 *
 * Unlike @ref findWorstCaseSensitivity / @ref findWorstCaseIAE, this variant has no
 * built-in notion of a controller - @p metric_fn is free to close its own loop (or
 * ignore feedback entirely) via capture.
 *
 * @param metric_fn Maps a spawned plant to a scalar metric; higher = worse.
 */
inline WorstCaseResult findWorstCase(
    std::function<StateSpace(const Eigen::VectorXd&)> plant_factory,
    std::function<double(const StateSpace&)> metric_fn,
    const Eigen::VectorXd& param_nominal,
    const Eigen::VectorXd& param_sigma,
    const Eigen::VectorXd& lower_bounds = {},
    const Eigen::VectorXd& upper_bounds = {},
    const WorstCaseSearchParams& p = {})
{
    auto metric = [&](const Eigen::VectorXd& phys) -> double
    {
        return metric_fn(plant_factory(phys));
    };
    return detail::runWorstCaseSearch(metric, param_nominal, param_sigma, lower_bounds, upper_bounds, p);
}

} // namespace ctrl

// Self-registration - included by ControllerToolbox.h umbrella.
#include "ControllerRegistry.h"
CTRL_REGISTER_FEATURE(worst_case_search)
