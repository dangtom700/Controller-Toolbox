#pragma once
#include "ControllerMonitor.h"   // for CUSUMChart (reused as-is)
#include <Eigen/Dense>
#include <cmath>
#include <optional>

/**
 * @file MismatchDetector.h
 * @brief Real-time model-plant mismatch detection via CUSUM on innovation sequence (D1).
 *
 * `MismatchDetector` runs a two-sided CUSUM chart on the normalised innovation
 * (prediction error) from a `KalmanFilter` or `MovingHorizonEstimator`.  A
 * persistent upward shift in the innovation magnitude indicates that the filter
 * model no longer describes the plant accurately - i.e., model-plant mismatch.
 *
 * **Algorithm (two-sided CUSUM on innovation RMS):**
 * @code
 *   v_rms[k] = ||innovation[k]|| / sqrt(p)    // p = measurement dimension
 *   C_plus[k]  = max(0, C_plus[k-1]  + v_rms[k] - sigma*(1 + k_cusum))
 *   C_minus[k] = max(0, C_minus[k-1] - v_rms[k] - sigma*(1 + k_cusum))
 *   Alarm: C_plus >= h_threshold*sigma  OR  C_minus >= h_threshold*sigma
 * @endcode
 *
 * **Typical usage (after enabling on a KalmanFilter):**
 * @code
 *   ctrl::MismatchDetectorParams dp;
 *   dp.sigma      = 0.1;   // expected innovation RMS when model is correct
 *   dp.k_cusum    = 0.5;   // standard CUSUM slack (half-sigma)
 *   dp.h_threshold = 5.0;  // fire after ~10-20 samples of sustained shift
 *
 *   kf.enableMismatchDetection(dp);
 *   // ...
 *   kf.step(y, u);
 *   if (kf.mismatchDetected())
 *       std::puts("[D1] Model-plant mismatch detected - consider re-estimation.");
 * @endcode
 *
 * **Tuning sigma:**
 * Run the filter with a known-good model for a period, collect innovation samples,
 * and set `sigma = sample_std(||innov||/sqrt(p))`.  For a correctly tuned KF the
 * innovation is white with zero mean; a well-tuned filter in steady state has
 * `||innov||/sqrt(p) approx = sqrt(trace(S)/p)` where `S = C*P*C' + R`.
 *
 * @see ControllerMonitor.h - CUSUMChart (same algorithm, different wrapping).
 * @see KalmanFilter.h     - enableMismatchDetection().
 * @see MovingHorizonEstimator.h - enableMismatchDetection().
 */

namespace ctrl {

/**
 * @brief Parameters for MismatchDetector.
 */
struct MismatchDetectorParams
{
    double sigma       = 1.0;   ///< Expected in-control innovation RMS (user must tune).
    double k_cusum     = 0.5;   ///< CUSUM slack in multiples of sigma (standard: 0.5).
    double h_threshold = 5.0;   ///< Detection threshold in multiples of sigma (standard: 4-5).
};

/**
 * @brief CUSUM-based model-plant mismatch detector for KF / MHE innovation sequences.
 *
 * Feed normalised innovations step-by-step via `update()`.  Query `detected()` and
 * `score()` at any time.  Does not depend on `IController` or observer infrastructure -
 * it is a plain stateful object that can also be used independently.
 */
class MismatchDetector
{
public:
    /** @brief Construct with optional parameters. */
    explicit MismatchDetector(const MismatchDetectorParams& p = {})
        : params_(p)
    {
        syncCusum();
    }

    /**
     * @brief Feed a scalar innovation sample (normalised by user).
     * @param scalar_innov  Normalised scalar innovation e.g. ||innov||/sqrt(p).
     */
    void update(double scalar_innov)
    {
        // Use absolute value so both sign conventions produce upward shifts.
        detected_ = cusum_.update(std::abs(scalar_innov));
    }

    /**
     * @brief Feed a vector innovation (computes ||v||/sqrt(p) internally).
     * @param innov  Innovation vector (p * 1).
     */
    void update(const Eigen::VectorXd& innov)
    {
        const double n = static_cast<double>(innov.size());
        update(innov.norm() / std::sqrt(n > 0.0 ? n : 1.0));
    }

    /**
     * @brief @c true if the CUSUM statistic has exceeded the detection threshold.
     *
     * Remains @c true until @c reset() is called; use @c score() to inspect magnitude.
     */
    bool detected() const { return detected_; }

    /**
     * @brief Current CUSUM statistic max(C+, C-) in raw units (not normalised by sigma).
     *
     * Compare against `h_threshold * sigma` for a threshold-relative reading.
     */
    double score() const { return cusum_.statistic(); }

    /** @brief Reset CUSUM accumulators and detection flag. */
    void reset()
    {
        cusum_.reset();
        detected_ = false;
    }

    /** @brief Update parameters (resets CUSUM to apply new thresholds cleanly). */
    void setParams(const MismatchDetectorParams& p)
    {
        params_ = p;
        syncCusum();
        reset();
    }

    /** @brief Read-only access to current parameters. */
    const MismatchDetectorParams& params() const { return params_; }

    /** @brief Direct access to the underlying CUSUMChart (read-only). */
    const CUSUMChart& cusum() const { return cusum_; }

private:
    void syncCusum()
    {
        cusum_.target = 0.0;             // innovation mean = 0 when consistent
        cusum_.sigma  = params_.sigma;
        cusum_.k      = params_.k_cusum;
        cusum_.h      = params_.h_threshold;
    }

    MismatchDetectorParams params_;
    CUSUMChart             cusum_;
    bool                   detected_ = false;
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(mismatch_detector)
