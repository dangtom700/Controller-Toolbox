// ex135_plc_adaptive_retuning.cpp - PLC-adaptive online retuning over a server/PLC link.
//
// Fusion: ctrl::AutoTuner (CMA-ES) + ctrl::RecursiveLeastSquares + ctrl::MismatchDetector
//         + ctrl::AtomicParamBuffer + ctrl::NetworkChannel
//
// Architecture:
//
//     PLC (slave)  : plant whose dynamics DRIFT mid-run; DiscretePID at the fast tick.
//                    Reads its gains from the AtomicParamBuffer every tick - an O(1) lock-free
//                    copy that never stalls the loop.
//        |  telemetry (y, u) -> NetworkChannel
//     srv (master) : RecursiveLeastSquares identifies the live plant from the telemetry.
//                    MismatchDetector watches the one-step-ahead RLS residual.
//                    On a sustained alarm the master runs a CMA-ES tuning session against the
//                    freshly identified model and publishes the new gains.
//
// The whole point is the separation of timescales. Identification and CMA-ES optimisation are
// expensive - the tuning session below costs hundreds of closed-loop simulations - and they run
// on the master's own schedule. The real-time loop never does more than a lock-free read(). That
// is precisely what ctrl::AtomicParamBuffer exists for, and ctrl::PIDParams is all-doubles, so
// it satisfies the template's trivially-copyable static_assert directly.
//
// The drift is a fouling/wear signature: the plant gain falls and its time constant lengthens,
// so gains tuned for the clean plant become far too sluggish. A fixed-gain arm and the adaptive
// arm see the IDENTICAL drift and the identical disturbance sequence, so the comparison measures
// the retuning rather than an easier scenario.
//
// Retuning is TRIGGERED BY THE DETECTOR, not scheduled at a hard-coded time. A demo that retunes
// on a timer would prove nothing about detection - it would just be a gain schedule.

#include <ControllerToolbox.h>
#include <AtomicParamBuffer.h>      // opt-in: not pulled in by the umbrella header

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <random>

namespace {

// -- Plant: first-order lag whose parameters drift ----------------------------
//   x' = -a*x + b*u
constexpr double kA_clean = 2.5;    // 1/tau before drift  (tau = 0.40 s)
constexpr double kB_clean = 2.5;    // unity DC gain
constexpr double kA_worn  = 0.7;    // 1/tau after drift   (tau = 1.43 s - much slower)
constexpr double kB_worn  = 1.0;    // DC gain falls to 0.4

constexpr double kTs     = 0.020;
constexpr double kTsim   = 90.0;
constexpr int    kNSteps = static_cast<int>(kTsim / kTs + 0.5);

constexpr double kCommissionTime = 20.0;  // freeze the nominal model here (plant still clean)
constexpr double kDriftStart     = 30.0;  // wear begins
constexpr double kDriftEnd       = 36.0;  // fully worn (ramped between, as real wear is)

constexpr double kLatency = 0.010;
constexpr double kJitter  = 0.003;
constexpr unsigned kSeed  = 90210u;

// Setpoint programme: alternating steps so both arms are continuously excited - RLS needs
// persistent excitation to identify anything, and a constant setpoint would starve it.
double referenceAt(double t)
{
    const int phase = static_cast<int>(t / 6.0) % 2;
    return phase == 0 ? 1.0 : 0.6;
}

double plantA(double t)
{
    if (t <= kDriftStart) return kA_clean;
    if (t >= kDriftEnd)   return kA_worn;
    const double s = (t - kDriftStart) / (kDriftEnd - kDriftStart);
    return kA_clean + s * (kA_worn - kA_clean);
}
double plantB(double t)
{
    if (t <= kDriftStart) return kB_clean;
    if (t >= kDriftEnd)   return kB_worn;
    const double s = (t - kDriftStart) / (kDriftEnd - kDriftStart);
    return kB_clean + s * (kB_worn - kB_clean);
}

ctrl::PIDParams initialGains()
{
    ctrl::PIDParams p;
    p.Kp = 1.20; p.Ki = 1.80; p.Kd = 0.02; p.N = 30.0;
    p.uMin = -6.0; p.uMax = 6.0; p.Kb = 1.0;
    return p;
}

/// Closed-loop IAE of a candidate gain set against an identified first-order model
/// x[k+1] = ad*x[k] + bd*u[k]. This is the CMA-ES cost function - it runs entirely on the
/// master, against the MODEL, never against the live plant.
double simulatedCost(const ctrl::PIDParams& cand, double ad, double bd)
{
    ctrl::DiscretePID pid(cand, kTs);
    double x = 0.0, iae = 0.0, u_prev = 0.0, du_sum = 0.0;
    const int N = 400;
    for (int k = 0; k < N; ++k) {
        const double ref = (k < N / 2) ? 1.0 : 0.6;     // a step in each direction
        const double u   = pid.compute(ref - x);
        x = ad * x + bd * u;
        if (!std::isfinite(x) || std::abs(x) > 1e3) return 1e6;   // reject unstable candidates
        iae    += std::abs(ref - x) * kTs;
        du_sum += std::abs(u - u_prev) * kTs;
        u_prev  = u;
    }
    return iae + 0.05 * du_sum;   // small effort penalty keeps the tuning from going bang-bang
}

struct ArmResult {
    double iae_total    = 0.0;
    double iae_postdrift = 0.0;
    int    retunes      = 0;
    double first_retune = -1.0;
    ctrl::PIDParams final_gains{};
    double id_a = 0.0, id_b = 0.0;   // last identified model
};

/// @param adaptive  true -> detector-triggered retuning; false -> gains frozen at the initial set
ArmResult runArm(bool adaptive)
{
    ArmResult r;
    r.final_gains = initialGains();

    // -- the real-time loop's gain channel ------------------------------------
    ctrl::AtomicParamBuffer<ctrl::PIDParams> gain_buf(initialGains());
    ctrl::DiscretePID pid(initialGains(), kTs);

    // -- master-side identification + drift detection -------------------------
    // TWO models, and the split is the whole trick:
    //
    //   rls (adaptive, lambda < 1) TRACKS the live plant and supplies the model AutoTuner tunes
    //   against. Its own residual is useless as a drift detector precisely because it adapts:
    //   at lambda = 0.99 the effective memory is 1/(1-lambda) = 100 samples = 2 s, so it absorbs
    //   a 6 s drift ramp and its residual never leaves the noise floor. An estimator cannot
    //   detect the drift it is busy absorbing.
    //
    //   nominal_* is a snapshot of that model FROZEN at commissioning. Scoring live data against
    //   the frozen model is what makes drift visible: the one-step prediction error grows as the
    //   plant walks away from its commissioned behaviour, and that is what the detector watches.
    //
    // ARX(1,1): y[k] = -a1*y[k-1] + b0*u[k-1], exactly the first-order lag structure.
    ctrl::RecursiveLeastSquares rls(1, 1, kTs, /*lambda=*/0.99, /*P0_scale=*/1e3);

    ctrl::MismatchDetectorParams mp;
    mp.sigma       = 0.005;  // frozen-model residual when the plant still matches commissioning
    mp.k_cusum     = 0.5;
    mp.h_threshold = 6.0;
    ctrl::MismatchDetector detector(mp);

    double nominal_ad = 0.0, nominal_bd = 0.0;
    bool   commissioned = false;
    double prev_y = 0.0, prev_u = 0.0;

    ctrl::NetworkChannelParams lp;
    lp.latency_mean = kLatency;
    lp.jitter_sigma = kJitter;
    lp.loss_prob    = 0.0;
    lp.seed         = kSeed;
    ctrl::NetworkChannel<double> up_y(lp), up_u(lp);

    std::mt19937 rng(kSeed);
    std::normal_distribution<double> meas_noise(0.0, 0.002);

    double x = 0.0;
    double y_rx = 0.0, u_rx = 0.0;
    double last_retune_t = -1e9;
    bool   alarm_latched = false;
    int    alarm_ticks   = 0;

    for (int k = 0; k < kNSteps; ++k) {
        const double t   = k * kTs;
        const double ref = referenceAt(t);

        // ---- PLC: lock-free gain read, then the control law -----------------
        const ctrl::PIDParams live = gain_buf.read();
        pid.setParams(live);
        const double u = pid.compute(ref - x);

        // ---- PLC -> master telemetry ----------------------------------------
        up_y.send(x + meas_noise(rng), t);
        up_u.send(u, t);
        double v = 0.0;
        if (up_y.tryReceive(v, t)) y_rx = v;
        if (up_u.tryReceive(v, t)) u_rx = v;

        // ---- master: adaptive identification (feeds the tuner) ---------------
        rls.update(y_rx, u_rx);

        const Eigen::VectorXd den_now = rls.denominator();   // [1, a1]
        const Eigen::VectorXd num_now = rls.numerator();     // [b0]
        const double ad_now = (den_now.size() > 1) ? -den_now(1) : 0.0;
        const double bd_now = (num_now.size() > 0) ?  num_now(0) : 0.0;

        // ---- master: commissioning snapshot, then drift scoring --------------
        if (!commissioned && t >= kCommissionTime) {
            nominal_ad   = ad_now;
            nominal_bd   = bd_now;
            commissioned = true;
        }
        if (commissioned) {
            // One-step prediction error of the FROZEN commissioned model on live data.
            const double y_pred = nominal_ad * prev_y + nominal_bd * prev_u;
            detector.update(y_rx - y_pred);
        }
        prev_y = y_rx;
        prev_u = u_rx;

        // ---- master: detector-triggered tuning session -----------------------
        if (adaptive) {
            if (detector.detected()) ++alarm_ticks; else alarm_ticks = 0;

            const bool sustained = alarm_ticks > 40;               // ~0.8 s of sustained alarm
            const bool cooled    = (t - last_retune_t) > 8.0;      // don't retune continuously
            if (sustained && cooled) {
                // Tune against the ADAPTIVE model - it reflects the plant as it is now, which
                // is the whole reason it is kept separate from the frozen commissioning model.
                const double ad = ad_now;
                const double bd = bd_now;

                // Only tune against a plausible, stable identified model.
                if (std::isfinite(ad) && std::isfinite(bd) &&
                    std::abs(ad) < 0.9999 && std::abs(bd) > 1e-6) {

                    // The Ki upper bound is BINDING at the tuned optimum (it lands on 12.0).
                    // That is deliberate and worth knowing rather than coincidence: the worn
                    // plant has a 1.43 s time constant and a DC gain of 1.43, so the
                    // integral action the cost function wants is genuinely large. The bound is
                    // a safety limit on how aggressive an unattended tuner may get, not a
                    // tuning artefact - raising it would buy a little more IAE and a lot less
                    // margin.
                    ctrl::AutoTunerParams atp;
                    atp.n       = 3;
                    atp.lower   = Eigen::Vector3d(0.05, 0.05, 0.0);
                    atp.upper   = Eigen::Vector3d(12.0, 12.0, 0.5);
                    atp.maxIter = 40;          // bounded: this is "background", not unbounded
                    atp.sigma0  = 0.3;
                    ctrl::AutoTuner tuner(atp, /*seed=*/1234u);

                    const ctrl::PIDParams base = gain_buf.read();
                    const ctrl::TunerResult res = tuner.tune(
                        [&](const Eigen::VectorXd& p) {
                            ctrl::PIDParams cand = base;
                            cand.Kp = p(0); cand.Ki = p(1); cand.Kd = p(2);
                            return simulatedCost(cand, ad, bd);
                        },
                        Eigen::Vector3d(base.Kp, base.Ki, base.Kd));

                    if (res.params.size() == 3 && res.params.allFinite()) {
                        ctrl::PIDParams tuned = base;
                        tuned.Kp = res.params(0);
                        tuned.Ki = res.params(1);
                        tuned.Kd = res.params(2);
                        gain_buf.publish(tuned);      // <- the RT loop picks this up next tick
                        r.final_gains = tuned;
                        r.id_a = ad; r.id_b = bd;
                        ++r.retunes;
                        if (r.first_retune < 0.0) r.first_retune = t;

                        // RE-BASELINE. Without this the frozen commissioning model stays
                        // permanently wrong after the drift, so the detector alarms forever and
                        // the loop retunes on every cooldown expiry - 7 sessions instead of the
                        // one the fault actually warrants. Accepting the new plant as the
                        // reference is what closes the detect -> adapt -> settle cycle.
                        nominal_ad = ad;
                        nominal_bd = bd;
                    }
                }
                last_retune_t = t;
                alarm_ticks   = 0;
                detector.reset();
                alarm_latched = true;
            }
        }

        // ---- plant, with drifting parameters ---------------------------------
        x += kTs * (-plantA(t) * x + plantB(t) * u);
        if (!std::isfinite(x)) x = 0.0;

        const double e = std::abs(ref - x);
        r.iae_total += e * kTs;
        if (t >= kDriftEnd) r.iae_postdrift += e * kTs;
    }

    (void)alarm_latched;
    return r;
}

}  // namespace

int main()
{
    std::printf("=== ex135: PLC-adaptive online retuning over a server/PLC link ===\n\n");
    std::printf("plant     : x' = -a x + b u, drifting a %.2f -> %.2f, b %.2f -> %.2f\n",
                kA_clean, kA_worn, kB_clean, kB_worn);
    std::printf("            (tau %.2f s -> %.2f s, DC gain %.2f -> %.2f) over t = %.0f..%.0f s\n",
                1.0 / kA_clean, 1.0 / kA_worn, kB_clean / kA_clean, kB_worn / kA_worn,
                kDriftStart, kDriftEnd);
    std::printf("tick      : %.0f ms, %d steps (%.0f s)\n", kTs * 1e3, kNSteps, kTsim);
    std::printf("master    : RLS(1,1) -> MismatchDetector -> AutoTuner (CMA-ES)"
                " -> AtomicParamBuffer\n");
    std::printf("initial   : Kp=%.3f Ki=%.3f Kd=%.3f\n\n",
                initialGains().Kp, initialGains().Ki, initialGains().Kd);

    const ArmResult fixed_arm = runArm(/*adaptive=*/false);
    const ArmResult adapt_arm = runArm(/*adaptive=*/true);

    std::printf("  %-20s %12s %16s %9s\n", "arm", "IAE total", "IAE post-drift", "retunes");
    std::printf("  %-20s %12.4f %16.4f %9d\n",
                "fixed gains", fixed_arm.iae_total, fixed_arm.iae_postdrift, fixed_arm.retunes);
    std::printf("  %-20s %12.4f %16.4f %9d\n",
                "adaptive retuning", adapt_arm.iae_total, adapt_arm.iae_postdrift,
                adapt_arm.retunes);

    std::printf("\n  first retune triggered at t = %.2f s (drift completes at %.0f s)\n",
                adapt_arm.first_retune, kDriftEnd);
    std::printf("  identified worn model: a_d = %.4f, b_d = %.4f"
                "  (true worn: a_d = %.4f, b_d = %.4f)\n",
                adapt_arm.id_a, adapt_arm.id_b,
                1.0 - kA_worn * kTs, kB_worn * kTs);
    std::printf("  gains  Kp %.3f -> %.3f | Ki %.3f -> %.3f | Kd %.3f -> %.3f\n",
                initialGains().Kp, adapt_arm.final_gains.Kp,
                initialGains().Ki, adapt_arm.final_gains.Ki,
                initialGains().Kd, adapt_arm.final_gains.Kd);

    const double improvement = (fixed_arm.iae_postdrift > 0.0)
        ? 100.0 * (fixed_arm.iae_postdrift - adapt_arm.iae_postdrift) / fixed_arm.iae_postdrift
        : 0.0;
    std::printf("  post-drift IAE improvement: %.1f %%\n", improvement);

    // -- acceptance -------------------------------------------------------------
    const double gain_delta = std::abs(adapt_arm.final_gains.Kp - initialGains().Kp) +
                              std::abs(adapt_arm.final_gains.Ki - initialGains().Ki);

    const bool retuned      = adapt_arm.retunes > 0;
    // Triggered by the DETECTOR, so it must fire after the drift starts - never on a timer.
    const bool triggered_ok = adapt_arm.first_retune > kDriftStart;
    const bool gains_moved  = gain_delta > 0.1;
    const bool better       = adapt_arm.iae_postdrift < 0.85 * fixed_arm.iae_postdrift;
    const bool finite_ok    = std::isfinite(adapt_arm.iae_total) &&
                              std::isfinite(fixed_arm.iae_total);

    std::printf("\n  retuning happened     = %s (%d session(s))\n",
                retuned ? "yes" : "no", adapt_arm.retunes);
    std::printf("  detector-triggered    = %s (t = %.2f s > drift start %.0f s)\n",
                triggered_ok ? "yes" : "no", adapt_arm.first_retune, kDriftStart);
    std::printf("  gains actually moved  = %s (|dKp|+|dKi| = %.3f > 0.1)\n",
                gains_moved ? "yes" : "no", gain_delta);
    std::printf("  beats fixed gains     = %s (%.4f < 85%% of %.4f)\n",
                better ? "yes" : "no", adapt_arm.iae_postdrift, fixed_arm.iae_postdrift);

    const bool ok = retuned && triggered_ok && gains_moved && better && finite_ok;
    std::printf("\n%s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
