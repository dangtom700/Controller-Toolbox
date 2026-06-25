#pragma once
#include "MuAnalysis.h"
#include "PlantModel.h"
#include "Features.h"
#include <Eigen/Dense>
#include <complex>
#include <vector>

/**
 * @file LFTSystem.h
 * @brief General multi-block LFT/Delta channel-gather for mu-analysis (Phase 3 RC1).
 *
 * Generalizes `MuAnalysis::peakMu()`'s hardcoded single canonical "M = sigma_rel * T"
 * multiplicative-output-uncertainty loop into arbitrary placement of one or more `Delta`
 * blocks against an open-loop map's own channels.
 *
 * **Key simplification (resolved during design):** `computeMu()`/`UncertaintyStructure`
 * never take `Delta`'s actual numeric realization, only its *structure* (block sizes/types)
 * - mu-analysis only needs the matrix `M` that `Delta` would multiply against, indexed into
 * the right block order. There is therefore no algebraic loop-closing/elimination to perform;
 * generalizing `peakMu()` is purely a *channel-gather* problem: given an open-loop map's full
 * frequency response, select and reorder the rows/columns each `Delta` block touches into the
 * canonical block-ordered matrix `computeMu()` expects.
 *
 * **Deviation from the roadmap sketch:** the constructor takes a plain `StateSpace` (`M0`,
 * the nominal open-loop map) rather than a `GeneralisedPlant` - `GeneralisedPlant`'s extra
 * control/measurement channels exist specifically for closing a *controller* loop
 * (`DiscreteHinf::solve()`'s job), which `LFTSystem` has no constructor parameter for at all.
 * A caller who needs both an active control loop and a multi-block uncertainty representation
 * closes the control loop first (e.g. via `DiscreteHinf`, or any fixed K), then hands the
 * resulting closed-loop `StateSpace` to `LFTSystem` as `M0` - the same two-stage pattern
 * `peakMu()` already uses internally (`gangOfFour` closes G/K, then scales `T`).
 *
 * @see docs/superpowers/specs/2026-06-24-lft-system-design.md
 */

namespace ctrl {

/**
 * @brief Per-block row/column placement of each `UncertaintyStructure` block within `M0`'s
 *        own frequency-response matrix.
 *
 * Block `i` occupies output rows `[rowStart[i], rowStart[i] + struc.blocks[i].r_out)` and
 * input columns `[colStart[i], colStart[i] + struc.blocks[i].r_in)` of `M0`'s `(p_out x m_in)`
 * frequency-response matrix. Blocks need not be contiguous or span `M0`'s full dimension -
 * channels not claimed by any block are simply excluded from the gathered matrix.
 */
struct LFTChannelMap
{
    std::vector<int> rowStart; ///< Per-block output-row offset into M0 (size == struc.blocks.size()).
    std::vector<int> colStart; ///< Per-block input-column offset into M0 (size == struc.blocks.size()).
};

/**
 * @brief General multi-block LFT channel-gather for mu-analysis.
 */
class LFTSystem
{
public:
    /**
     * @brief Construct from the nominal open-loop map, uncertainty structure, and channel map.
     * @param M0    Open-loop StateSpace (rows = output channels Delta blocks read from,
     *              cols = input channels Delta blocks write into).
     * @param struc Block-diagonal uncertainty structure.
     * @param map   Per-block row/column placement within M0.
     * @throws std::invalid_argument If `map.rowStart`/`colStart` don't have one entry per
     *         `struc.blocks`, if any block's range falls outside M0's output/input dimensions,
     *         or if any two blocks' ranges overlap.
     */
    LFTSystem(const StateSpace &M0, const UncertaintyStructure &struc, const LFTChannelMap &map);

    /**
     * @brief Gathered M(jw) at each frequency, block-ordered to match the uncertainty
     *        structure - directly feedable to `computeMu()`.
     * @param omegas Frequencies [rad/s].
     * @return One `struc.totalOutputs() x struc.totalInputs()` matrix per frequency.
     */
    std::vector<Eigen::MatrixXcd> closedLoopFreqResponse(const std::vector<double> &omegas) const;

    /**
     * @brief Peak structured singular value over a log-spaced frequency grid.
     *
     * Convenience wrapper mirroring the free-function `peakMu()`'s own signature, minus
     * `sigma_rel` (bake any relative uncertainty scaling into `M0` or per-block data before
     * construction instead - see the header's "Deviation" note).
     * @param freq_points Log-spaced frequency grid size.
     * @param omega_min   Minimum frequency [rad/s].
     */
    PeakMuResult peakMu(int freq_points = 200, double omega_min = 1e-2) const;

private:
    StateSpace          M0_;
    UncertaintyStructure struc_;
    LFTChannelMap        map_;
    std::vector<int>     rowIdx_; ///< Flattened, block-ordered row-gather index list.
    std::vector<int>     colIdx_; ///< Flattened, block-ordered column-gather index list.
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(lft_system)
