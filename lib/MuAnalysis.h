#pragma once
#include "PlantModel.h"
#include <Eigen/Dense>
#include <vector>
#include <complex>

/**
 * @file MuAnalysis.h
 * @brief Structured singular value (mu) analysis for block-structured uncertainty.
 *
 * Implements the standard D-scaling upper bound on the structured singular value
 * mu_Delta(M) for a block-diagonal uncertainty structure Delta = blkdiag(Delta_1, ..., Delta_F):
 * @code
 *   mu_Delta(M) <= min_{D in scaling commutant} sigma_max(D * M * D^-1)
 * @endcode
 *
 * **Scope (V1):**
 *  - @c ComplexFull blocks: the scaling commutant is exactly scalar*Identity, so the
 *    per-block-scalar D-iteration implemented here is the textbook-exact upper bound.
 *  - @c ComplexScalar (repeated complex scalar) blocks: also scaled with a single
 *    positive scalar per block. This is a *valid* upper bound (any D in a subset of
 *    the true commutant still satisfies the inequality above) but is only *tight*
 *    when the block size is 1; for size > 1 the textbook-tight bound requires a full
 *    Hermitian-matrix scaling per block, which is not implemented here.
 *  - @c RealScalar blocks require an additional G-scaling (Packard & Doyle 1993) and
 *    are **not implemented** - @ref computeMu throws if any block has this type.
 *  - The lower bound (when requested) is the generic, always-valid `rho(M) <= mu(M)`
 *    spectral-radius bound - not the full power-iteration lower bound (NP-hard in
 *    general; deferred per the implementation plan).
 *
 * @see Packard, A. & Doyle, J. "The complex structured singular value." Automatica,
 *      29(1), 71-109 (1993).
 * @see docs/robust_implementation_plan.md, Phase 3.
 */

namespace ctrl
{

/**
 * @brief One block of a block-diagonal structured uncertainty Delta.
 */
struct UncertaintyBlock
{
    /// @brief Uncertainty type. RealScalar is accepted in the struct but rejected
    /// at @ref computeMu time (G-scaling not implemented).
    enum class Type { RealScalar, ComplexScalar, ComplexFull };

    Type type  = Type::ComplexFull; ///< Block type.
    int  r_out = 1;                 ///< Rows of this block (output side of Delta_i).
    int  r_in  = 1;                 ///< Columns of this block (input side of Delta_i).
};

/**
 * @brief Full block-diagonal uncertainty structure Delta = blkdiag(blocks).
 *
 * The interconnection matrix M passed to @ref computeMu must be square with
 * `M.rows() == totalOutputs()` and `M.cols() == totalInputs()`.
 */
struct UncertaintyStructure
{
    std::vector<UncertaintyBlock> blocks;

    /// @brief Sum of r_in over all blocks (Delta's output dimension / M's column count).
    int totalInputs() const;

    /// @brief Sum of r_out over all blocks (Delta's input dimension / M's row count).
    int totalOutputs() const;
};

/**
 * @brief Upper (and optionally lower) bound on the structured singular value at one frequency.
 */
struct MuBound
{
    double upper = 0.0; ///< D-scaling upper bound (always computed, always valid).
    double lower = 0.0; ///< Spectral-radius lower bound rho(M) (only if requested; 0 otherwise).
};

/**
 * @brief Compute mu bounds at each supplied frequency-response matrix.
 *
 * @param M_freq Frequency response of the interconnection matrix M(jw) (e.g. from
 *               @ref freqResponseGrid), one square matrix per frequency point.
 * @param struc  Block-diagonal uncertainty structure; `struc.totalOutputs()` must equal
 *               `struc.totalInputs()` and both must equal each matrix's dimension.
 * @param compute_lower_bound If true, also computes the spectral-radius lower bound
 *               `rho(M) <= mu(M)` (cheap, generic, not the full power-iteration bound).
 * @return One @ref MuBound per entry of @p M_freq.
 * @throws std::invalid_argument If a matrix's dimensions don't match @p struc, if M is
 *         not square, or if @p struc contains a @c RealScalar block.
 */
std::vector<MuBound>
computeMu(const std::vector<Eigen::MatrixXcd>& M_freq,
          const UncertaintyStructure& struc,
          bool compute_lower_bound = false);

/**
 * @brief Peak mu over a frequency grid for a unity-feedback loop under multiplicative
 *        output uncertainty, plus the full per-frequency curve.
 */
struct PeakMuResult
{
    MuBound peak;                     ///< Bound at the frequency where upper is largest.
    double  peak_omega_rad_s = 0.0;   ///< Frequency [rad/s] at which the peak occurs.
    std::vector<MuBound> mu_curve;    ///< Full per-frequency bound (same length as the grid).
};

/**
 * @brief Peak structured singular value of a closed loop under scaled multiplicative
 *        output uncertainty.
 *
 * Builds the interconnection matrix as `M(jw) = sigma_rel * T(jw)`, where `T` is the
 * complementary sensitivity of the unity-feedback loop closing @p G and @p K (see
 * @ref SystemAnalysis::gangOfFour) - the standard M-Delta formulation for output
 * multiplicative uncertainty `G_actual = (I + sigma_rel * Delta) * G` (Skogestad &
 * Postlethwaite, *Multivariable Feedback Control*, Ch. 8). For a single @c ComplexFull
 * block spanning the whole output space this reduces exactly to `sigma_rel * ||T||_inf`.
 *
 * @param G          Plant.
 * @param K          Controller (closes the loop with @p G; same convention as gangOfFour).
 * @param struc      Uncertainty structure (must size-match the plant output dimension).
 * @param sigma_rel  Relative uncertainty magnitude (the loop is robust iff peak mu < 1).
 * @param freq_points Log-spaced frequency grid size (default 200).
 * @param omega_min  Minimum frequency [rad/s] (default 0.01).
 * @return PeakMuResult with the worst-case bound, its frequency, and the full curve.
 * @throws std::invalid_argument On dimension/sample-time mismatch (from gangOfFour) or
 *         a structure size mismatch (from computeMu).
 */
PeakMuResult
peakMu(const StateSpace& G, const StateSpace& K,
       const UncertaintyStructure& struc,
       double sigma_rel   = 0.1,
       int    freq_points = 200,
       double omega_min   = 1e-2);

/**
 * @brief Largest @p sigma_rel for which the loop remains robustly stable, found by
 *        bisection on @ref peakMu's peak upper bound.
 *
 * Bisects `sigma_rel in [0, sigma_max]` for the boundary where `peakMu(...).peak.upper`
 * crosses 1. If even @p sigma_max keeps the peak below 1, returns @p sigma_max directly
 * (the true radius is at least that large; the caller may re-run with a bigger bound).
 *
 * @param G            Plant.
 * @param K             Controller.
 * @param struc         Uncertainty structure.
 * @param sigma_max     Upper end of the bisection search (default 2.0).
 * @param bisect_iters  Number of bisection iterations (default 30).
 * @return The robust stability radius (largest safe @c sigma_rel).
 */
double
robustStabilityRadius(const StateSpace& G, const StateSpace& K,
                       const UncertaintyStructure& struc,
                       double sigma_max    = 2.0,
                       int    bisect_iters = 30);

} // namespace ctrl

// Self-registration - included by ControllerToolbox.h umbrella.
#include "ControllerRegistry.h"
CTRL_REGISTER_FEATURE(mu_analysis)
