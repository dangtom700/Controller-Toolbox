#include "ControllerTuner.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <numeric>

namespace ctrl
{

    // ============================================================
    // RelayAutoTuner
    // ============================================================

    RelayAutoTuner::RelayAutoTuner(const RelayTunerConfig &cfg, double sampleTime)
        : cfg_(cfg), Ts_(sampleTime)
    {
    }

    double RelayAutoTuner::step(double y)
    {
        step_cnt_++;

        // Relay logic with hysteresis
        double relay_out;
        if (relayHigh_ && y > cfg_.hysteresis)
        {
            relayHigh_ = false;
            relay_out = -cfg_.relayAmplitude;
            collecting_ = true; // ignore transient before first switch
        }
        else if (!relayHigh_ && y < -cfg_.hysteresis)
        {
            relayHigh_ = true;
            relay_out = cfg_.relayAmplitude;
            collecting_ = true;
        }
        else
        {
            relay_out = relayHigh_ ? cfg_.relayAmplitude : -cfg_.relayAmplitude;
        }

        // Detect zero crossings after transient (sign change detection)
        if (collecting_ && (y * y_prev_ < 0.0))
            crossTimes_.push_back(step_cnt_ * Ts_);

        // Track extrema ONLY after transient; do NOT reset between crossings.
        // We accumulate global peak/trough over all stable cycles.
        if (collecting_)
        {
            if (y > peak_pos_)
                peak_pos_ = y;
            if (y < peak_neg_)
                peak_neg_ = y;
        }

        y_prev_ = y;

        // Need 2.cyclesRequired crossings for stable period estimate
        const int needed = 2 * cfg_.cyclesRequired;
        if (!done_ && static_cast<int>(crossTimes_.size()) >= needed)
        {
            // Average period over last cyclesRequired full cycles
            const int n = static_cast<int>(crossTimes_.size());
            Tu_ = 2.0 * (crossTimes_[n - 1] - crossTimes_[n - 1 - cfg_.cyclesRequired]) / cfg_.cyclesRequired;

            // Ku from accumulated peak-to-peak amplitude across all stable cycles
            const double a_y = (peak_pos_ - peak_neg_) / 2.0;
            if (a_y > 1e-12)
                Ku_ = (4.0 * cfg_.relayAmplitude) / (M_PI * a_y);

            done_ = true;
        }

        return relay_out;
    }

    PIDParams RelayAutoTuner::computePIDParams(PIDTuningRule rule, double lambda) const
    {
        if (!done_)
            throw std::runtime_error("RelayAutoTuner: call isDone() first.");

        PIDParams p;
        switch (rule)
        {
        case PIDTuningRule::ZieglerNichols:
            p.Kp = 0.60 * Ku_;
            p.Ki = p.Kp * 2.0 / Tu_;
            p.Kd = p.Kp * Tu_ / 8.0;
            break;

        case PIDTuningRule::TyreusLuyben:
            p.Kp = Ku_ / 2.2;
            p.Ki = p.Kp / (2.2 * Tu_);
            p.Kd = p.Kp * Tu_ / 6.3;
            break;

        case PIDTuningRule::AMIGO:
            // Simplified AMIGO (Åström 2006) from relay test data alone
            p.Kp = 0.40 * Ku_;
            p.Ki = 0.80 * Ku_ / Tu_;
            p.Kd = 0.10 * p.Kp * Tu_;
            break;

        case PIDTuningRule::IMC:
            // IMC from relay test: approximate tau approx = Tu/(2pi), theta approx = Ts
            {
                const double tau_est = Tu_ / (2.0 * M_PI);
                const double lam = (lambda > 0) ? lambda : tau_est;
                p.Kp = (2.0 * tau_est) / (Ku_ * lam);
                p.Ki = p.Kp / tau_est;
                p.Kd = 0.0;
            }
            break;
        }

        p.N = 10.0 / (Tu_ / (2.0 * M_PI)); // derivative filter: cutoff ~ 10* nominal BW
        p.Kb = std::sqrt(p.Ki * p.Kd + 1e-12);
        return p;
    }

    // ============================================================
    // StepResponseTuner
    // ============================================================

    StepResponseTuner::FOPDTModel
    StepResponseTuner::identify(const std::vector<double> &time,
                                const std::vector<double> &output,
                                double stepMagnitude)
    {
        if (time.size() < 4 || time.size() != output.size())
            throw std::invalid_argument("StepResponseTuner: need at least 4 matched samples.");

        const double y_inf = output.back();
        const double K = y_inf / stepMagnitude;

        // Find indices where output crosses 28.3 % and 63.2 % of final value (Smith 1972 tangent)
        const double y283 = 0.283 * y_inf;
        const double y632 = 0.632 * y_inf;

        double t283 = 0.0, t632 = 0.0;
        bool got283 = false, got632 = false;

        for (size_t i = 1; i < output.size(); ++i)
        {
            if (!got283 && output[i] >= y283)
            {
                t283 = time[i];
                got283 = true;
            }
            if (!got632 && output[i] >= y632)
            {
                t632 = time[i];
                got632 = true;
            }
            if (got283 && got632)
                break;
        }

        if (!got283 || !got632)
            throw std::runtime_error("StepResponseTuner: output did not reach 63.2 % of steady state.");

        // Process-reaction-curve method (Åström & Hägglund):
        //   τ = 1.5.(t63.2 - t28.3),   θ = t63.2 - τ
        const double tau = 1.5 * (t632 - t283);
        const double theta = std::max(0.0, t632 - tau);

        return {K, tau, theta};
    }

    PIDParams StepResponseTuner::computePIDParams(const FOPDTModel &m,
                                                  double Ts,
                                                  PIDTuningRule rule,
                                                  double lambda)
    {
        PIDParams p;
        const double lam = (lambda > 0) ? lambda : 0.5 * m.tau;

        switch (rule)
        {
        case PIDTuningRule::ZieglerNichols:
            p.Kp = 1.2 * m.tau / (m.K * m.theta);
            p.Ki = p.Kp / (2.0 * m.theta);
            p.Kd = p.Kp * 0.5 * m.theta;
            break;

        case PIDTuningRule::TyreusLuyben:
            p.Kp = 0.45 * m.tau / (m.K * m.theta);
            p.Ki = p.Kp / (2.63 * m.theta);
            p.Kd = 0.0;
            break;

        case PIDTuningRule::IMC:
            // Rivera, Morari & Skogestad (1986) IMC-PID
            p.Kp = (2.0 * m.tau + m.theta) / (2.0 * m.K * (lam + m.theta));
            p.Ki = p.Kp / (m.tau + m.theta / 2.0);
            p.Kd = p.Kp * m.tau * m.theta / (2.0 * m.tau + m.theta);
            break;

        case PIDTuningRule::AMIGO:
            // AMIGO (Åström & Hägglund 2006) - optimised for robustness
            {
                const double ratio = m.theta / m.tau;
                p.Kp = (1.0 / m.K) * (0.2 + 0.45 * m.tau / m.theta);
                p.Ki = p.Kp * (0.4 * m.theta + 0.8 * m.tau) /
                       (m.theta + 0.1 * m.tau) / (m.tau + 0.8 * m.theta);
                p.Kd = p.Kp * 0.5 * m.tau * m.theta / (0.3 * m.tau + m.theta);
                (void)ratio;
            }
            break;
        }

        p.N = 10.0 / m.theta; // derivative filter ~10* dead-time frequency
        p.Kb = std::sqrt(std::abs(p.Ki * p.Kd) + 1e-12);
        return p;
    }

    // ============================================================
    // LQRWeightTuner
    // ============================================================

    LQRParams LQRWeightTuner::brysonMethod(const Eigen::VectorXd &maxStateDeviation,
                                           const Eigen::VectorXd &maxControlEffort)
    {
        LQRParams lp;
        lp.Q = (1.0 / maxStateDeviation.array().square()).matrix().asDiagonal();
        lp.R = (1.0 / maxControlEffort.array().square()).matrix().asDiagonal();
        return lp;
    }

    LQRParams LQRWeightTuner::polePlacementHint(
        const StateSpace &plant,
        const std::vector<std::complex<double>> &desiredPoles,
        int maxIter)
    {
        // Start from Bryson with unit bounds, then iterate Q scaling to steer poles.
        const int n = plant.stateSize();
        const int m = plant.inputSize();

        Eigen::VectorXd sx = Eigen::VectorXd::Ones(n);
        Eigen::VectorXd su = Eigen::VectorXd::Ones(m);
        LQRParams lp = brysonMethod(sx, su);

        for (int iter = 0; iter < maxIter; ++iter)
        {
            DiscreteLQR lqr(plant, lp);
            Eigen::MatrixXd Acl = plant.A - plant.B * lqr.gainMatrix();
            Eigen::VectorXcd ev = Acl.eigenvalues();

            // Compute RMS pole-location residual
            double err = 0.0;
            for (int i = 0; i < n; ++i)
            {
                double best = 1e9;
                for (auto &dp : desiredPoles)
                    best = std::min(best, std::abs(ev(i) - dp));
                err += best * best;
            }
            err = std::sqrt(err / n);
            if (err < 1e-4)
                break;

            // Scale Q by gradient heuristic: increase Q if poles are too slow
            for (int i = 0; i < n; ++i)
            {
                if (std::abs(ev(i)) > std::abs(desiredPoles[i % desiredPoles.size()]))
                    lp.Q(i, i) *= 1.1;
                else
                    lp.Q(i, i) *= 0.95;
            }
        }
        return lp;
    }

    // ============================================================
    // MPCHorizonTuner
    // ============================================================

    double MPCHorizonTuner::estimateSettlingTime(const StateSpace &plant, int maxSteps)
    {
        const int p = plant.outputSize();
        const int m = plant.inputSize();

        Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());
        Eigen::VectorXd u = Eigen::VectorXd::Ones(m);
        Eigen::VectorXd y_ss;

        // Run to approximate steady state
        for (int k = 0; k < maxSteps; ++k)
            y_ss = plant.C * x + plant.D * u, x = plant.A * x + plant.B * u;

        // Restart and find 99 % settling
        x.setZero();
        const double y_final = y_ss(0);
        const double thr = 0.99 * y_final;

        for (int k = 0; k < maxSteps; ++k)
        {
            Eigen::VectorXd y = plant.C * x + plant.D * u;
            x = plant.A * x + plant.B * u;
            if (std::abs(y(0)) >= std::abs(thr))
                return k * plant.Ts;
        }
        return maxSteps * plant.Ts;
    }

    MPCHorizonTuner::Recommendation
    MPCHorizonTuner::recommend(const StateSpace &plant,
                               double Ts,
                               double rho_y,
                               double rho_u)
    {
        const double ts = estimateSettlingTime(plant);
        const int Np = std::max(5, static_cast<int>(std::ceil(ts / Ts)));
        const int Nc = std::max(1, Np / 3);
        return {Np, Nc, rho_y, rho_u, ts};
    }

    // ============================================================
    // ZieglerNicholsTuner
    // ============================================================

    PIDParams ZieglerNicholsTuner::tuneImpl(const Input &in)
    {
        PIDParams p;
        p.Kp = 0.6 * in.Ku;
        p.Ki = p.Kp * 2.0 / in.Tu;
        p.Kd = p.Kp * in.Tu / 8.0;
        p.N = 20.0;
        p.Kb = std::sqrt(std::abs(p.Ki * p.Kd) + 1e-12);
        return p;
    }

    // ============================================================
    // CohenCoonTuner
    // ============================================================

    PIDParams CohenCoonTuner::tuneImpl(const StepResponseTuner::FOPDTModel &m, double /*Ts*/)
    {
        if (m.theta < 1e-9)
            throw std::invalid_argument(
                "CohenCoonTuner: dead time theta must be > 0."
                " Cohen-Coon requires finite dead time.");

        const double r = m.theta / m.tau;
        const double Kp = (m.tau / (m.K * m.theta)) * (4.0 / 3.0 + r / 4.0);
        const double Ti = m.theta * (32.0 + 6.0 * r) / (13.0 + 8.0 * r);
        const double Td = 4.0 * m.theta / (11.0 + 2.0 * r);

        PIDParams p;
        p.Kp = Kp;
        p.Ki = Kp / Ti;
        p.Kd = Kp * Td;
        p.N = 10.0 / m.theta;
        p.Kb = std::sqrt(std::abs(p.Ki * p.Kd) + 1e-12);
        return p;
    }

    // ============================================================
    // LoopShapingTuner
    // ============================================================

    LeadLagParams LoopShapingTuner::tuneImpl(const Input &in)
    {
        const double phase_rad = in.phase_add_deg * M_PI / 180.0;

        if (phase_rad <= 0.0 || phase_rad >= M_PI / 2.0)
        {
            std::clog << "[LoopShapingTuner] Warning: phase_add_deg must be in (0, 90)."
                         " Returning a unity-gain compensator at the requested crossover.\n";
            const double K = (in.gain_at_wc > 1e-12) ? 1.0 / in.gain_at_wc : 1.0;
            return {in.omega_c * 0.1, in.omega_c * 10.0, K};
        }

        const double beta = std::sin(phase_rad);
        const double alpha = (1.0 + beta) / (1.0 - beta); // p / z
        const double sqrtA = std::sqrt(alpha);

        LeadLagParams lp;
        lp.continuousZero = in.omega_c / sqrtA; // z = omega_c / sqrt(alpha)
        lp.continuousPole = in.omega_c * sqrtA; // p = omega_c * sqrt(alpha)
        lp.gain = (in.gain_at_wc > 1e-12)
                      ? sqrtA / in.gain_at_wc // K = sqrt(alpha) / |G(jomega_c)|
                      : sqrtA;
        return lp;
    }

    // ============================================================
    // KalmanWeightTuner
    // ============================================================

    KalmanNoiseParams KalmanWeightTuner::fromNoise(const Eigen::VectorXd &maxProcessNoise,
                                                   const Eigen::VectorXd &maxMeasNoise)
    {
        KalmanNoiseParams kp;
        kp.Qf = maxProcessNoise.array().square().matrix().asDiagonal();
        kp.Rf = maxMeasNoise.array().square().matrix().asDiagonal();
        kp.P0 = kp.Qf;
        return kp;
    }

    KalmanNoiseParams KalmanWeightTuner::isotropic(int nStates,
                                                   int nOutputs,
                                                   double sigmaProcess,
                                                   double sigmaMeas)
    {
        KalmanNoiseParams kp;
        kp.Qf = Eigen::MatrixXd::Identity(nStates, nStates) * (sigmaProcess * sigmaProcess);
        kp.Rf = Eigen::MatrixXd::Identity(nOutputs, nOutputs) * (sigmaMeas * sigmaMeas);
        kp.P0 = kp.Qf;
        return kp;
    }

} // namespace ctrl
