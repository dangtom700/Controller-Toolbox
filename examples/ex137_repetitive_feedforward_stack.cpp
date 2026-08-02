// ex137_repetitive_feedforward_stack.cpp - Which feedforward mechanism, and when.
//
// Fusion: ctrl::designZPETC + ctrl::LearningFeedforwardController (which owns the ctrl::ILC)
//         + ctrl::RepetitiveController, layered over a ctrl::DiscretePID
//
// ZPETC (ex59), ILC (ex70), learning feedforward (ex129) and repetitive control (ex29) have
// each had a solo example for a long time and have never been combined. docs/fusion_opportunity_
// backlog.md item B1 proposed stacking all four on a pick-and-place axis on the grounds that
// they "compose naturally".
//
// THEY DO NOT, and measuring that is this demo's actual result.
//
// ZPETC and ILC are SUBSTITUTES, not complements: both attack the tracking error that repeats
// trial to trial, one from a model and one from data. Whatever the model already explains, ILC
// does not get to learn. RepetitiveController is the odd one out - it is the only mechanism here
// that touches a DIFFERENT error class, and outside that class it contributes nothing.
//
// So the demo is a 2x2 of error classes rather than a ladder of layers:
//
//                              | no periodic disturbance | strong periodic disturbance
//     -------------------------+-------------------------+----------------------------
//     accurate nominal model   | ZPETC alone is enough   | ZPETC + RC
//     poor nominal model       | ZPETC + ILC             | all three
//
// Every cell is measured, for all five stack combinations, on the identical trajectory and
// disturbance sequence. The interesting entries are the ones where a mechanism is applied
// OUTSIDE its class and earns nothing - that is the part a "just add more feedforward" design
// argument gets wrong, and it is asserted here rather than described.
//
// The role split between ILC and RC is engineered, not assumed. The trial is 400 steps; the
// disturbance period is 157 steps. 400/157 is not an integer, so the disturbance advances 0.548
// of a cycle every trial and never presents the same trial-indexed waveform twice. ILC learns a
// feedforward indexed by step-within-trial, so it cannot represent that signal at all - its
// Q-filter correctly forgets it as noise. RepetitiveController, whose buffer is indexed modulo
// 157, sees a perfectly stationary target. Give both the same period and this collapses into
// two mechanisms learning the same thing and fighting over the credit.
//
// Two composition notes:
//
//   1. B1 lists ILC and LearningFeedforwardController as separate components. They are not:
//      LearningFeedforwardController OWNS an ILC and runs the whole trial state machine (step
//      index, wrap, updateFeedforward() at the boundary, record-only trials). Constructing a
//      second bare ILC alongside it would duplicate the update law. Same shape as ex133's
//      KalmanFilter-embeds-MismatchDetector finding. The ILC is reachable via .ilc().
//
//   2. The ZPETC prefilter is applied by hand, not through a ControllerStack in Additive mode.
//      A stack feeds one scalar to every entry, but the feedforward path needs the REFERENCE r
//      while the feedback path needs the ERROR r - y. Two different signals, so the
//      two-degree-of-freedom structure has to be wired explicitly.

#include <ControllerToolbox.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

namespace {

constexpr double kTs = 0.005;              // 200 Hz servo
constexpr int    kTrialLen = 400;          // 2.0 s pick-and-place stroke
constexpr int    kTrials   = 40;
constexpr int    kDistPeriod = 157;        // deliberately NOT a divisor of kTrialLen

constexpr double kLoad = 0.15;             // constant gravity load - repeats every trial
constexpr double kUMin = -3.0, kUMax = 3.0;

// Scenario axes.
constexpr double kModelGood = 0.90;        // nominal gain / true gain
constexpr double kModelPoor = 0.50;
constexpr double kDistOff   = 0.00;
constexpr double kDistOn    = 0.10;

// Learning gains. Both were found by sweeping against this plant, and both are bounded by
// stability rather than by taste:
//   ILC  Lp = 0.05  - P-type ILC diverged at Lp >= 0.20 here (RMS 0.63 -> 2.2 -> 2.5). The
//                     plant's direct feedthrough (0.020) is far smaller than its later Markov
//                     parameters, which is the classic ill-conditioning that makes P-type ILC
//                     grow a transient before it converges. ILC::Params also clamps Lp to [0,1].
//   RC   Krc = 0.02 - at Krc = 0.10 the repetitive loop diverged outright (RMS 1.88), and even
//                     at the stable Krc = 0.05 it went from harmless to actively harmful when
//                     applied with no periodic disturbance present (0.0096 -> 0.0431, 4.5x
//                     worse). Conservative learning gains are not timidity here.
constexpr double kILC_Lp = 0.05, kILC_Q = 0.95;
constexpr double kRC_Krc = 0.02, kRC_Q  = 0.99;

// -- Plant ------------------------------------------------------------------------------
// TRUE axis, with a non-minimum-phase zero at z = -1.2 (the usual consequence of sampling a
// resonant mechanism fast). NMP is what makes ZPETC interesting: a direct inverse would place
// a pole at -1.2 and diverge, so the prefilter cancels only the minimum-phase factor and
// normalises the rest at DC.
ctrl::TransferFunction truePlant()
{
    return ctrl::TransferFunction({0.020, 0.0240}, {1.0, -1.850, 0.860}, kTs);
}

/// Nominal model handed to designZPETC. `gain_ratio` < 1 makes it under-predict the true gain;
/// the pole shift is held fixed. This is the knob that decides whether ZPETC or ILC does the
/// work, and the whole point of running both extremes.
ctrl::TransferFunction nominalPlant(double gain_ratio)
{
    constexpr double dp = 0.02;
    return ctrl::TransferFunction({0.020 * gain_ratio, 0.0240 * gain_ratio},
                                  {1.0, -1.850 - dp, 0.860 + dp}, kTs);
}

ctrl::PIDParams servoTuning()
{
    ctrl::PIDParams p;
    p.Kp = 0.50; p.Ki = 4.00; p.Kd = 0.010; p.N = 50.0;
    p.uMin = kUMin; p.uMax = kUMax; p.Kb = 1.0;
    return p;
}

// -- Reference: a smooth raised-cosine stroke, held at both ends ------------------------
// Smoothness is not cosmetic. The ZPETC prefilter contains the plant's A(z) polynomial, which
// differences the reference; a step reference would make it spike into saturation and the
// comparison would measure clipping rather than feedforward quality.
double strokeRef(int k)
{
    constexpr int kRise = 120, kHold = 80, kFall = 120;
    if (k < kRise)                       // 0 -> 1
        return 0.5 * (1.0 - std::cos(M_PI * k / kRise));
    if (k < kRise + kHold)               // hold high
        return 1.0;
    if (k < kRise + kHold + kFall) {     // 1 -> 0
        const int j = k - (kRise + kHold);
        return 0.5 * (1.0 + std::cos(M_PI * j / kFall));
    }
    return 0.0;                          // hold low
}

// -- Stack selection: a bitmask so all five combinations share one loop ------------------
enum Layer : unsigned { kZPETC = 1u, kILC = 2u, kRC = 4u };

struct RunResult {
    double rms_final = 0.0;    // RMS tracking error over the LAST trial
    double rms_first = 0.0;    // ... over the first, to show learning happened at all
    bool   finite    = true;
};

RunResult run(unsigned stack, double model_gain, double dist_amp, double Krc = kRC_Krc)
{
    const ctrl::StateSpace plant = ctrl::tf2ss(truePlant());
    const ctrl::StateSpace nom   = ctrl::tf2ss(nominalPlant(model_gain));

    std::shared_ptr<ctrl::IController> ctl =
        std::make_shared<ctrl::DiscretePID>(servoTuning(), kTs);

    // ILC first, so the repetitive controller plugs in AROUND the whole learning loop - the
    // "plug-in around an already-stabilising controller" structure RepetitiveController documents.
    if (stack & kILC) {
        ctrl::ILC::Params ip;
        ip.N  = kTrialLen;
        ip.Ts = kTs;
        ip.mode = ctrl::ILC::Mode::PType;
        ip.Lp = kILC_Lp;
        ip.Q_filter = kILC_Q;        // forgetting: what does not repeat per trial decays away
        ip.uMin = kUMin; ip.uMax = kUMax;

        ctrl::LearningFFParams lp;
        lp.trialLength = kTrialLen;
        lp.learnTrials = 1;          // trial 0 records only
        lp.uMin = kUMin; lp.uMax = kUMax;

        ctl = std::make_shared<ctrl::LearningFeedforwardController>(ctl, ip, lp, kTs);
    }
    if (stack & kRC) {
        ctrl::RepetitiveParams rp;
        rp.periodSteps = kDistPeriod;
        rp.Krc = Krc;
        rp.Q   = kRC_Q;
        rp.uMin = kUMin; rp.uMax = kUMax;
        ctl = std::make_shared<ctrl::RepetitiveController>(ctl, rp, kTs);
    }

    ctrl::ZPETCResult zp = ctrl::designZPETC(nom);
    Eigen::VectorXd x_ff    = Eigen::VectorXd::Zero(zp.filter.A.rows());
    Eigen::VectorXd x_plant = Eigen::VectorXd::Zero(plant.A.rows());

    RunResult r;
    double y = 0.0;

    for (int trial = 0; trial < kTrials; ++trial) {
        double sq = 0.0;
        for (int k = 0; k < kTrialLen; ++k) {
            const int    k_global = trial * kTrialLen + k;
            const double ref      = strokeRef(k);

            // -- feedforward path: driven by the REFERENCE ---------------------------
            double u_ff = 0.0;
            if (stack & kZPETC) {
                Eigen::VectorXd rv(1); rv(0) = ref;
                u_ff = ctrl::ssStep(zp.filter, x_ff, rv)(0);
            }

            // -- feedback path: driven by the ERROR ----------------------------------
            const double e = ref - y;
            const double u = std::clamp(u_ff + ctl->compute(e), kUMin, kUMax);

            // -- plant, constant load, and the non-trial-synchronised disturbance -----
            const double d_per = dist_amp * std::sin(2.0 * M_PI * k_global / kDistPeriod);
            Eigen::VectorXd uv(1); uv(0) = u + kLoad + d_per;
            y = ctrl::ssStep(plant, x_plant, uv)(0);

            if (!std::isfinite(y) || !std::isfinite(u)) { r.finite = false; return r; }
            sq += e * e;
        }
        const double rms = std::sqrt(sq / kTrialLen);
        if (trial == 0)           r.rms_first = rms;
        if (trial == kTrials - 1) r.rms_final = rms;
    }
    return r;
}

struct Cell {
    const char *name;
    double model_gain, dist_amp;
    double pid, zpetc, zpetc_ilc, zpetc_rc, all;
    bool   finite = true;
};

Cell measure(const char *name, double model_gain, double dist_amp)
{
    Cell c{name, model_gain, dist_amp, 0, 0, 0, 0, 0, true};
    const RunResult r0 = run(0,                     model_gain, dist_amp);
    const RunResult r1 = run(kZPETC,                model_gain, dist_amp);
    const RunResult r2 = run(kZPETC | kILC,         model_gain, dist_amp);
    const RunResult r3 = run(kZPETC | kRC,          model_gain, dist_amp);
    const RunResult r4 = run(kZPETC | kILC | kRC,   model_gain, dist_amp);
    c.pid = r0.rms_final; c.zpetc = r1.rms_final; c.zpetc_ilc = r2.rms_final;
    c.zpetc_rc = r3.rms_final; c.all = r4.rms_final;
    c.finite = r0.finite && r1.finite && r2.finite && r3.finite && r4.finite;
    return c;
}

}  // namespace

int main()
{
    std::printf("=== ex137: which feedforward mechanism, and when ===\n\n");
    std::printf("axis      : NMP servo, %d Hz, trial = %d steps (%.1f s), %d trials per run\n",
                static_cast<int>(1.0 / kTs), kTrialLen, kTrialLen * kTs, kTrials);
    std::printf("load      : constant %.2f - repeats every trial, so ILC can learn it\n", kLoad);
    std::printf("disturb.  : %.2f sin(2 pi k / %d) - the period does NOT divide the %d-step\n",
                kDistOn, kDistPeriod, kTrialLen);
    std::printf("            trial, so it slips %.3f of a cycle per trial and ILC provably\n",
                static_cast<double>(kTrialLen % kDistPeriod) / kDistPeriod);
    std::printf("            cannot represent it, while RC (buffer mod %d) sees it as stationary\n",
                kDistPeriod);
    std::printf("gains     : ILC Lp = %.2f (diverges at 0.20), RC Krc = %.2f (diverges at 0.10)\n\n",
                kILC_Lp, kRC_Krc);

    // -- ZPETC design diagnostics ---------------------------------------------------------
    const ctrl::ZPETCResult zp = ctrl::designZPETC(ctrl::tf2ss(nominalPlant(kModelGood)));
    std::printf("designZPETC on the nominal model:\n");
    std::printf("  transmission zeros : ");
    for (const auto &z : zp.zeros)
        std::printf("%.3f%+.3fj (|z| = %.3f)  ", z.real(), z.imag(), std::abs(z));
    std::printf("\n  non-minimum phase  : %s", zp.hasNMPZeros ? "yes" : "no");
    if (zp.hasNMPZeros)
        std::printf(" (%zu NMP zero(s) - handled by DC normalisation, not cancellation)",
                    zp.nmpZeros.size());
    std::printf("\n  DC amplitude error : %.4f  (the NMP zero sits near z = -1, so the composite\n"
                "                       response collapses at Nyquist; harmless for a smooth\n"
                "                       reference, fatal for a stepped one)\n\n",
                zp.dcAmplitudeError);

    // -- the 2x2 -----------------------------------------------------------------------
    const Cell good_quiet = measure("accurate model, no periodic", kModelGood, kDistOff);
    const Cell poor_quiet = measure("poor model,     no periodic", kModelPoor, kDistOff);
    const Cell good_dist  = measure("accurate model, periodic   ", kModelGood, kDistOn);
    const Cell poor_dist  = measure("poor model,     periodic   ", kModelPoor, kDistOn);
    const Cell *cells[4]  = {&good_quiet, &poor_quiet, &good_dist, &poor_dist};

    std::printf("final-trial RMS tracking error, identical trajectory and disturbance per column:\n\n");
    std::printf("  %-28s %9s %9s %11s %10s %10s\n",
                "scenario", "PID", "+ZPETC", "+ZPETC+ILC", "+ZPETC+RC", "all three");
    for (const Cell *c : cells)
        std::printf("  %-28s %9.5f %9.5f %11.5f %10.5f %10.5f\n",
                    c->name, c->pid, c->zpetc, c->zpetc_ilc, c->zpetc_rc, c->all);

    std::printf("\nmarginal contribution of each mechanism (negative = it helped):\n\n");
    std::printf("  %-28s %14s %14s %14s\n",
                "scenario", "ZPETC vs PID", "ILC vs ZPETC", "RC vs ZPETC");
    for (const Cell *c : cells)
        std::printf("  %-28s %13.0f%% %13.0f%% %13.0f%%\n", c->name,
                    100.0 * (c->zpetc / c->pid - 1.0),
                    100.0 * (c->zpetc_ilc / c->zpetc - 1.0),
                    100.0 * (c->zpetc_rc / c->zpetc - 1.0));

    // -- misapplication cost --------------------------------------------------------------
    // RC at the conservative gain is merely inert without a periodic disturbance. Push the
    // learning gain up for faster convergence and inert becomes harmful - worth measuring,
    // because "it did not help" and "it made things worse" call for different responses.
    const RunResult rc_hot = run(kZPETC | kRC, kModelGood, kDistOff, 0.05);
    std::printf("\n  RC applied with nothing to cancel: Krc = %.2f leaves RMS at %.5f (inert,\n"
                "  vs %.5f without it); raising Krc to 0.05 for faster learning degrades it to\n"
                "  %.5f - a %.1fx penalty for using the right tool on the wrong error class.\n",
                kRC_Krc, good_quiet.zpetc_rc, good_quiet.zpetc,
                rc_hot.rms_final, rc_hot.rms_final / good_quiet.zpetc);

    // -- acceptance -------------------------------------------------------------------------
    // Each assertion states one claim of the error-class thesis. They are deliberately
    // two-sided: a mechanism must both EARN its place inside its class and FAIL to earn one
    // outside it, otherwise the "matched to an error class" claim is untested in one direction.
    const bool all_finite = good_quiet.finite && poor_quiet.finite &&
                            good_dist.finite && poor_dist.finite;

    // ZPETC owns model-explainable error: decisive with a good model, useless with a bad one.
    const bool zpetc_owns  = good_quiet.zpetc < 0.25 * good_quiet.pid;
    const bool zpetc_needs_model = poor_quiet.zpetc > 0.90 * poor_quiet.pid;

    // ILC covers exactly what the model did not: big win when the model is poor, nothing when
    // the error is periodic-but-not-trial-synchronised.
    const bool ilc_recovers = poor_quiet.zpetc_ilc < 0.75 * poor_quiet.zpetc;
    const bool ilc_blind    = good_dist.zpetc_ilc > 0.95 * good_dist.zpetc;

    // RC owns the periodic class, and only that class.
    const bool rc_owns  = good_dist.zpetc_rc < 0.50 * good_dist.zpetc;
    const bool rc_inert = good_quiet.zpetc_rc > 0.95 * good_quiet.zpetc;

    // Only the both-error-classes cell justifies carrying all three.
    const bool stack_pays = poor_dist.all < 0.85 * poor_dist.zpetc_rc &&
                            poor_dist.all < 0.62 * poor_dist.pid;

    std::printf("\n  ZPETC owns model error   = %s (accurate model: %.5f < 25%% of PID's %.5f)\n",
                zpetc_owns ? "yes" : "no", good_quiet.zpetc, good_quiet.pid);
    std::printf("  ...and needs that model  = %s (poor model: %.5f still > 90%% of PID's %.5f)\n",
                zpetc_needs_model ? "yes" : "no", poor_quiet.zpetc, poor_quiet.pid);
    std::printf("  ILC recovers the rest    = %s (poor model: %.5f < 75%% of ZPETC's %.5f)\n",
                ilc_recovers ? "yes" : "no", poor_quiet.zpetc_ilc, poor_quiet.zpetc);
    std::printf("  ...but is blind to phase = %s (periodic: %.5f, no better than ZPETC's %.5f)\n",
                ilc_blind ? "yes" : "no", good_dist.zpetc_ilc, good_dist.zpetc);
    std::printf("  RC owns periodic error   = %s (periodic: %.5f < 50%% of ZPETC's %.5f)\n",
                rc_owns ? "yes" : "no", good_dist.zpetc_rc, good_dist.zpetc);
    std::printf("  ...and nothing else      = %s (no periodic: %.5f, no better than %.5f)\n",
                rc_inert ? "yes" : "no", good_quiet.zpetc_rc, good_quiet.zpetc);
    std::printf("  full stack only pays     = %s (both classes: %.5f < 85%% of ZPETC+RC's %.5f\n"
                "                                  and < 62%% of PID's %.5f)\n",
                stack_pays ? "yes" : "no", poor_dist.all, poor_dist.zpetc_rc, poor_dist.pid);

    const bool ok = all_finite && zpetc_owns && zpetc_needs_model && ilc_recovers &&
                    ilc_blind && rc_owns && rc_inert && stack_pays;
    std::printf("\n%s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
