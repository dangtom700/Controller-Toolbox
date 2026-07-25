#pragma once
#include "ControllerRegistry.h"
#include "IController.h"
#include "IterativeLearningControl.h"
#include <Eigen/Dense>
#include <memory>
#include <string>

/**
 * @file LearningFeedforwardController.h
 * @brief Iterative-learning feedforward layered on a nominal feedback controller.
 *
 * Packages the two-phase ILC pattern that repeating-task studies re-implement by hand:
 * @code
 *   trial 0 .. learnTrials-1 : u[k] = nominal.compute(e[k])                  (record only)
 *   trial learnTrials ..     : u[k] = ilc.feedforward(k) + nominal.compute(e[k])
 * @endcode
 *
 * The trial state machine - step index, wrap at `trialLength`, the
 * `ILC::updateFeedforward()` call at the trial boundary, and the switch from
 * recording to applying - lives here instead of in each caller.
 *
 * **Usage:**
 * @code
 *   ctrl::ILC::Params ip;   ip.N = 200;  ip.Ts = Ts;  ip.Lp = 0.5;
 *   ctrl::LearningFFParams lp;  lp.trialLength = 200;  lp.learnTrials = 1;
 *   ctrl::LearningFeedforwardController lff(pid, ip, lp, Ts);
 *
 *   for (int k = 0; k < 5 * 200; ++k)          // 5 trials of the same trajectory
 *       u = lff.compute(r[k % 200] - y);       // trials advance automatically
 * @endcode
 *
 * **Sign handling.** `error` is forwarded to the nominal controller unchanged. The
 * error handed to `ILC::recordError()` is negated when the nominal controller reports
 * `TrackingErrorYMinusR` (DiscreteSMC and friends), because ILC's update law assumes
 * e = r - y and would otherwise learn a feedforward of the wrong sign.
 *
 * @note ILC only converges when the reference and disturbance repeat trial-to-trial;
 *       on a non-repeating task the learned term is noise. `lastRMSError()` per trial
 *       is the signal to watch.
 *
 * @see ILC - the underlying update law (P-type, D-type, norm-optimal).
 * @see Bristow, D.A., Tharayil, M. & Alleyne, A.G. (2006). A survey of iterative
 *      learning control. IEEE Control Systems Magazine 26(3), 96-114.
 */

namespace ctrl
{

/** @brief Trial-management parameters for @ref LearningFeedforwardController. */
struct LearningFFParams
{
    int trialLength = 200;  ///< Steps per trial; must equal ILC::Params::N.
    int learnTrials = 1;    ///< Recording-only trials before the feedforward is applied; >= 0.
    bool autoAdvance = true;///< Call ILC::updateFeedforward() automatically at each trial boundary.
    double uMin = -1e9;     ///< Lower saturation limit on the total command.
    double uMax = 1e9;      ///< Upper saturation limit on the total command.
};

/**
 * @brief Nominal feedback plus a trial-to-trial learned feedforward.
 */
class LearningFeedforwardController : public IController
{
public:
    /**
     * @brief Construct from a nominal controller and an ILC configuration.
     * @param nominal Feedback controller run every step.
     * @param ilc_p   ILC parameters; `ilc_p.N` must equal `params.trialLength`.
     * @param params  Trial-management and saturation settings.
     * @param Ts      Sample time [s].
     * @throws std::invalid_argument If nominal is null, Ts <= 0, uMin >= uMax,
     *         learnTrials < 0, or trialLength != ilc_p.N.
     */
    LearningFeedforwardController(std::shared_ptr<IController> nominal,
                                  const ILC::Params &ilc_p,
                                  const LearningFFParams &params,
                                  double Ts);

    // ---- IController -------------------------------------------------------

    /**
     * @brief Advance the nominal loop, record the error, and add the learned term.
     * @param error Tracking error, in the nominal controller's own convention.
     * @return Saturated total command u[k].
     */
    double compute(double error) override;

    /** @brief Mirrors the nominal controller - `error` is forwarded to it unchanged. */
    SignConvention signConvention() const override { return nominal_->signConvention(); }

    void reset() override;
    double sampleTime() const override { return Ts_; }
    std::string name() const override { return "LearningFeedforwardController"; }

    bool isHealthy() const override { return nominal_->isHealthy(); }
    bool hasInternalAntiWindup() const override { return nominal_->hasInternalAntiWindup(); }

    void bumplessInit(double u_target, double error) override;

    // ---- Trial interface ---------------------------------------------------

    /**
     * @brief Close the current trial manually (only needed when autoAdvance is false).
     *
     * Runs `ILC::updateFeedforward()` and rewinds the step index to 0.
     */
    void endTrial();

    /** @brief Number of completed trials (equals ILC::trialIndex()). */
    int trialIndex() const noexcept { return ilc_.trialIndex(); }
    /** @brief Step index within the current trial, in [0, trialLength). */
    int stepIndex() const noexcept { return k_; }
    /** @brief true while still in a recording-only trial (no feedforward applied). */
    bool learning() const noexcept { return ilc_.trialIndex() < p_.learnTrials; }
    /** @brief RMS tracking error of the last completed trial. */
    double lastRMSError() const noexcept { return ilc_.lastRMSError(); }

    // ---- Diagnostics -------------------------------------------------------

    /** @brief Learned feedforward contribution from the last compute() call. */
    double feedforwardTerm() const noexcept { return u_ff_; }
    /** @brief Last applied command u[k-1]. */
    double lastOutput() const noexcept { return u_prev_; }
    /** @brief Read-only access to the underlying ILC. */
    const ILC &ilc() const noexcept { return ilc_; }
    /** @brief Mutable access to the nominal controller. */
    const std::shared_ptr<IController> &nominalController() const noexcept { return nominal_; }

private:
    std::shared_ptr<IController> nominal_;
    ILC ilc_;
    LearningFFParams p_;
    double Ts_;

    bool flip_record_ = false; ///< true when the nominal uses e = y - r; ILC needs r - y.
    int k_ = 0;                ///< Step index within the current trial.
    double u_ff_ = 0.0;        ///< Last learned feedforward contribution.
    double u_prev_ = 0.0;      ///< Previous command u[k-1] (NaN-guard hold value).

    mutable Eigen::VectorXd notify_buf_{Eigen::VectorXd::Constant(1, 0.0)};
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(learning_feedforward)
