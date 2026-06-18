#pragma once
#include "PlantModel.h"
#include <vector>
#include <cstdint>

/**
 * @file RobustnessAnalysis.h
 * @brief Monte-Carlo robustness analysis for discrete-time closed loops.
 *
 * Spawns an ensemble of perturbed plant models (state-space or transfer-function),
 * closes each one around a fixed pre-designed controller, and aggregates closed-loop
 * robustness metrics (stability probability, gain/phase margins, peak sensitivity,
 * step-response IAE/settling/overshoot, nu-gap from nominal).
 *
 * Reuses:
 *   - SystemAnalysis::getPoles / calculateHInfinityNorm / calculateMargins
 *   - GapMetric::nuGap
 *   - PlantModel::ssStep
 *
 * The closed loop is unity negative feedback with the controller acting on the
 * tracking error e = r - y and driving the plant input u:
 * @code
 *   u = K(e),   y = G(u),   e = r - y
 *   S = (I + G*K)^{-1}        (sensitivity, maps r -> e)
 *   T = G*K*(I + G*K)^{-1}    (complementary sensitivity, maps r -> y, = I - S)
 * @endcode
 *
 * Phase 1 of the robustness-analysis roadmap (docs/robust_implementation_plan.md).
 *
 * @see SystemAnalysis.h, GapMetric.h
 */

namespace ctrl
{

/**
 * @brief Closed-loop robustness metrics from one sampled (perturbed) plant.
 *
 * Margins are NaN for MIMO plants (calculateMargins is SISO only) or when no
 * crossover is found. Time-domain metrics are NaN when the closed loop is
 * unstable (settling is NaN when the step never enters the settling band, and
 * overshoot is NaN when the step never exceeds its target).
 */
struct RobustnessSample
{
    int    sample_id        = 0;     ///< Index of this sample within the ensemble.
    bool   is_stable        = false; ///< All closed-loop poles strictly inside the unit disk.
    double gain_margin_db   = 0.0;   ///< Gain margin [dB]; NaN if MIMO or no crossover.
    double phase_margin_deg = 0.0;   ///< Phase margin [deg]; NaN if MIMO or no crossover.
    double hinf_sensitivity = 0.0;   ///< Peak sensitivity ||S||_inf.
    double hinf_comp_sens   = 0.0;   ///< Peak complementary sensitivity ||T||_inf.
    double iae              = 0.0;   ///< Step-response integral absolute error; inf if unstable.
    double settling_time_s  = 0.0;   ///< 5%-band settling time [s]; NaN if not settled.
    double overshoot_pct    = 0.0;   ///< Percent overshoot; NaN if no overshoot.
    double nu_gap_from_nominal = 0.0; ///< nuGap(nominal, this sample) in [0, 1].
};

/**
 * @brief Aggregated robustness statistics over a full ensemble.
 */
struct MonteCarloResult
{
    int    n_samples               = 0;   ///< Total samples evaluated.
    int    n_unstable              = 0;   ///< Samples with at least one closed-loop pole on/outside the unit disk.
    double instability_probability = 0.0; ///< n_unstable / n_samples.

    /// @brief Summary statistics for one scalar metric across the ensemble.
    /// Computed only over finite values; `worst` is the min for margins and the
    /// max for sensitivity / IAE / settling / nu-gap.
    struct MetricStats
    {
        double mean    = 0.0;
        double std_dev = 0.0;
        double p5      = 0.0;
        double p25     = 0.0;
        double p50     = 0.0;
        double p75     = 0.0;
        double p95     = 0.0;
        double worst   = 0.0;
    };

    MetricStats gm_stats;                    ///< Gain margin [dB] (worst = min).
    MetricStats pm_stats;                    ///< Phase margin [deg] (worst = min).
    MetricStats sensitivity_peak_stats;      ///< ||S||_inf (worst = max).
    MetricStats comp_sensitivity_peak_stats; ///< ||T||_inf (worst = max).
    MetricStats iae_stats;                   ///< Step IAE (worst = max).
    MetricStats settling_stats;              ///< Settling time [s] (worst = max).
    MetricStats nu_gap_stats;                ///< nu-gap from nominal (worst = max).

    std::vector<RobustnessSample> samples;   ///< Full per-sample record.
};

/**
 * @brief Per-parameter uncertainty description (reserved for config-driven runners).
 *
 * Not consumed by the scalar-sigma spawn_*() overloads below; provided for the
 * forthcoming config/analysis.json runner and worst-case search.
 */
struct PerturbationSpec
{
    enum class Distribution { Uniform, Normal };
    Distribution dist        = Distribution::Normal; ///< Sampling distribution.
    double       sigma       = 0.05; ///< Relative std-dev (Normal) or half-width (Uniform).
    double       lower_bound = 0.0;  ///< Absolute lower clamp (0 = no clamp).
    double       upper_bound = 0.0;  ///< Absolute upper clamp (0 = no clamp).
};

// ---------------------------------------------------------------------------
// Model spawning
// ---------------------------------------------------------------------------

/**
 * @brief Spawn perturbed transfer functions by relative Gaussian noise on coefficients.
 *
 * Each numerator coefficient b_i and each non-leading denominator coefficient a_i
 * (i >= 1) is scaled by (1 + sigma_i * N(0,1)). The leading denominator term den[0]
 * is held at 1 to keep the result monic.
 *
 * @param nominal           Nominal transfer function (z^-1 form, monic den).
 * @param num_samples       Number of perturbed models to generate.
 * @param numerator_sigma   Relative std-dev per numerator coefficient. Length must equal
 *                          num.size() or be 1 (applied uniformly).
 * @param denominator_sigma Relative std-dev per denominator coefficient (den[0] ignored).
 *                          Length must equal den.size() or be 1.
 * @param seed              RNG seed for reproducibility.
 * @return Vector of num_samples perturbed transfer functions.
 * @throws std::invalid_argument On sigma-vector length mismatch.
 */
std::vector<TransferFunction>
spawn_TF_samples(const TransferFunction& nominal,
                 uint32_t num_samples,
                 const std::vector<double>& numerator_sigma,
                 const std::vector<double>& denominator_sigma,
                 uint32_t seed = 42);

/**
 * @brief Spawn perturbed state-space models by element-wise relative Gaussian noise.
 *
 * Each entry M(i,j) of A, B, C, D is scaled independently by (1 + sigma_M * N(0,1)).
 * Structural zeros stay zero (multiplicative perturbation). Pass sigma = 0.0 to hold
 * a matrix fixed - sigma_B = sigma_C = sigma_D = 0 perturbs only A, the common
 * state-matrix-uncertainty case.
 *
 * @param nominal     Nominal discrete-time state-space model.
 * @param num_samples Number of perturbed models to generate.
 * @param sigma_A     Relative std-dev on A entries.
 * @param sigma_B     Relative std-dev on B entries (default 0).
 * @param sigma_C     Relative std-dev on C entries (default 0).
 * @param sigma_D     Relative std-dev on D entries (default 0).
 * @param seed        RNG seed for reproducibility.
 * @return Vector of num_samples perturbed state-space models.
 */
std::vector<StateSpace>
spawn_SS_samples(const StateSpace& nominal,
                 uint32_t num_samples,
                 double sigma_A,
                 double sigma_B = 0.0,
                 double sigma_C = 0.0,
                 double sigma_D = 0.0,
                 uint32_t seed  = 42);

// ---------------------------------------------------------------------------
// Monte-Carlo analysis
// ---------------------------------------------------------------------------

/**
 * @brief Evaluate one (plant, controller) pair: close the loop and measure robustness.
 *
 * Builds the closed-loop sensitivity S and complementary sensitivity T directly
 * (no per-step controller simulation) for stability/poles and Hinf norms, then
 * simulates the r->y map with ssStep for time-domain metrics.
 *
 * @param sample_id      Index recorded into the returned sample.
 * @param plant          Perturbed plant G (same dims/Ts as controller and nominal).
 * @param controller_ss  Pre-designed controller K in state-space form (error -> input).
 * @param nominal_plant  Reference plant for the nu-gap computation.
 * @param step_amplitude Reference step magnitude for time-domain metrics.
 * @param sim_duration_s Simulation horizon for time-domain metrics [s].
 * @param settling_band  Fractional settling band (0.05 = 5%).
 * @return Populated RobustnessSample.
 */
RobustnessSample
evaluateSample(int sample_id,
               const StateSpace& plant,
               const StateSpace& controller_ss,
               const StateSpace& nominal_plant,
               double step_amplitude = 1.0,
               double sim_duration_s = 50.0,
               double settling_band  = 0.05);

/**
 * @brief Run the closed-loop evaluation over an entire ensemble and aggregate.
 *
 * @param ensemble       Perturbed plants (typically from spawn_SS_samples).
 * @param controller_ss  Fixed controller state-space.
 * @param nominal_plant  Reference plant for nu-gap.
 * @param step_amplitude Reference step magnitude.
 * @param sim_duration_s Simulation horizon [s].
 * @return Aggregated MonteCarloResult including the full per-sample record.
 */
MonteCarloResult
runMonteCarlo(const std::vector<StateSpace>& ensemble,
              const StateSpace& controller_ss,
              const StateSpace& nominal_plant,
              double step_amplitude = 1.0,
              double sim_duration_s = 50.0);

/**
 * @brief Convenience wrapper: spawn an ensemble then run the Monte-Carlo analysis.
 *
 * The nominal plant is used both as the spawn centre and the nu-gap reference.
 *
 * @param nominal_plant  Nominal plant G.
 * @param controller_ss  Fixed controller K.
 * @param num_samples    Ensemble size.
 * @param sigma_A        Relative std-dev on A.
 * @param sigma_B        Relative std-dev on B (default 0).
 * @param sigma_C        Relative std-dev on C (default 0).
 * @param sigma_D        Relative std-dev on D (default 0).
 * @param seed           RNG seed.
 * @return Aggregated MonteCarloResult.
 */
MonteCarloResult
monteCarloAnalysis(const StateSpace& nominal_plant,
                   const StateSpace& controller_ss,
                   uint32_t num_samples,
                   double sigma_A,
                   double sigma_B = 0.0,
                   double sigma_C = 0.0,
                   double sigma_D = 0.0,
                   uint32_t seed  = 42);

} // namespace ctrl

// Self-registration - included by ControllerToolbox.h umbrella.
#include "ControllerRegistry.h"
CTRL_REGISTER_FEATURE(robustness_analysis)
