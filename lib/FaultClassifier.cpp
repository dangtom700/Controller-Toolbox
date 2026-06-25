#include "FaultClassifier.h"
#include <algorithm>
#include <cmath>

namespace ctrl
{

namespace
{
double stddevOf(const Eigen::VectorXd &v)
{
    const double m = v.mean();
    return std::sqrt((v.array() - m).square().sum() / static_cast<double>(v.size()));
}
} // namespace

FaultClassifier::FaultClassifier(const FaultDetectorParams &p) : p_(p)
{
    const int n = std::max(p_.confirm_window, 1);
    innovHist_ = Eigen::VectorXd::Zero(n);
    uHist_ = Eigen::VectorXd::Zero(n);
    yHist_ = Eigen::VectorXd::Zero(n);
}

FaultType FaultClassifier::classify(double innovation, double u_cmd, double y_meas)
{
    const int n = static_cast<int>(innovHist_.size());
    innovHist_(head_) = innovation;
    uHist_(head_) = u_cmd;
    yHist_(head_) = y_meas;
    head_ = (head_ + 1) % n;
    count_ = std::min(count_ + 1, n);

    if (count_ < n) return FaultType::None;

    const double rmsInnov = std::sqrt(innovHist_.array().square().mean());
    if (rmsInnov < p_.residual_threshold) return FaultType::None;

    const double meanInnov = innovHist_.mean();
    const double sigmaInnov = std::max(stddevOf(innovHist_), 1e-12);

    // Read out in chronological order (oldest..newest); head_ currently points at the oldest
    // entry (the slot that was just overwritten with the newest sample, then advanced past).
    Eigen::VectorXd uChrono(n), yChrono(n);
    for (int i = 0; i < n; ++i)
    {
        const int idx = (head_ + i) % n;
        uChrono(i) = uHist_(idx);
        yChrono(i) = yHist_(idx);
    }
    const Eigen::VectorXd du = uChrono.tail(n - 1) - uChrono.head(n - 1);
    const Eigen::VectorXd dy = yChrono.tail(n - 1) - yChrono.head(n - 1);

    const double duStd = stddevOf(du);
    const double dyStd = stddevOf(dy);

    const double uStd = stddevOf(uHist_);
    if (uStd < p_.stuck_du_threshold) return FaultType::ActuatorStuck;

    // corr(du_cmd, dy_meas) only speaks to actuator-vs-sensor causality when the command is
    // actually moving (duStd above the noise floor) - if it isn't, there is nothing to correlate
    // against either way, and falling through to the residual-amplitude check below is correct
    // (not "low correlation"; the bare 1e-12 div-by-zero guard used to conflate the two, making a
    // perfectly healthy but quiescent closed loop misclassify as an actuator fault). When the
    // command IS moving but the output doesn't respond at all (dyStd collapses to ~0), that *is*
    // the textbook broken-causal-link actuator-loss signature - decide it directly rather than
    // dividing by a near-zero dyStd.
    if (duStd > p_.stuck_du_threshold)
    {
        if (dyStd <= p_.stuck_du_threshold) return FaultType::ActuatorLoss;

        const double duMean = du.mean();
        const double dyMean = dy.mean();
        const double corr = ((du.array() - duMean) * (dy.array() - dyMean)).sum() /
                             (static_cast<double>(du.size()) * duStd * dyStd);
        if (std::fabs(corr) < p_.corr_threshold) return FaultType::ActuatorLoss;
    }

    return (std::fabs(meanInnov) > p_.bias_threshold * sigmaInnov) ? FaultType::SensorBias
                                                                     : FaultType::SensorNoise;
}

void FaultClassifier::reset()
{
    innovHist_.setZero();
    uHist_.setZero();
    yHist_.setZero();
    count_ = 0;
    head_ = 0;
}

} // namespace ctrl
